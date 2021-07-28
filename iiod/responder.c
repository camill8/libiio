// SPDX-License-Identifier: LGPL-2.1-or-later
/*
 * libiio - Library for interfacing industrial I/O (IIO) devices
 *
 * Copyright (C) 2021 Analog Devices, Inc.
 * Author: Paul Cercueil <paul.cercueil@analog.com>
 */

#include "debug.h"
#include "ops.h"
#include "thread-pool.h"

#include "../iiod-responder.h"

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define ARRAY_SIZE(x) (sizeof(x) ? sizeof(x) / sizeof((x)[0]) : 0)

static void responder_free_buf(void *d, ssize_t ret)
{
	free(d);
}

static void handle_print(struct parser_pdata *pdata,
			 const struct iiod_command *cmd,
			 struct iiod_command_data *cmd_data,
			 struct iiod_reader *reader)
{
	struct iiod_buf buf;

	if (pdata->xml_zstd) {
		buf.ptr = (void *) pdata->xml_zstd;
		buf.size = pdata->xml_zstd_len;

		iiod_reader_send_response(reader, pdata->xml_zstd_len, &buf, 1);
	} else {
		iiod_reader_send_response_code(reader, -EINVAL);
	}
}

static void handle_timeout(struct parser_pdata *pdata,
			   const struct iiod_command *cmd,
			   struct iiod_command_data *cmd_data,
			   struct iiod_reader *reader)
{
	struct iio_context *ctx = pdata->ctx;
	int ret;

	ret = iio_context_set_timeout(pdata->ctx, cmd->code);
	iiod_reader_send_response_code(reader, ret);
}

static void handle_open(struct parser_pdata *pdata,
			const struct iiod_command *cmd,
			struct iiod_command_data *cmd_data)
{
	uint32_t *mask, nb_words, sample_count = cmd->code;
	struct iio_context *ctx = pdata->ctx;
	struct iio_device *dev;
	struct iiod_buf cmd_buf;
	struct iiod_reader *reader;
	int ret = -ENXIO;

	reader = iiod_command_create_reader(cmd, cmd_data);
	if (!reader)
		return; /* We can't send an error code without "reader" */

	dev = iio_context_get_device(ctx, cmd->dev);
	if (!dev)
		goto err_free_reader;

	nb_words = (iio_device_get_channels_count(dev) + 31) / 32;

	mask = calloc(nb_words, sizeof(*mask));
	if (!mask) {
		ret = -ENOMEM;
		goto err_free_reader;
	}

	cmd_buf.ptr = mask;
	cmd_buf.size = nb_words * sizeof(*mask);

	ret = (int) iiod_command_data_read(cmd_data, &cmd_buf);
	if (ret < 0) {
		IIO_ERROR("Failed to read command data: %d\n", ret);
		goto err_free_mask;
	}

	reader = iiod_command_create_reader(cmd, cmd_data);
	if (!reader) {
		ret = -ENOMEM;
		goto err_free_mask;
	}

	ret = open_dev_helper(pdata, dev, sample_count, mask, nb_words,
			      cmd->op == IIOD_OP_OPEN_CYCLIC, reader);
	if (ret < 0) {
		IIO_ERROR("Failed to open device: %d\n", ret);
		goto err_free_reader;
	}

	iiod_reader_send_response_code(reader, ret);

	return;

err_free_mask:
	free(mask);
err_free_reader:
	iiod_reader_send_response_code(reader, ret);
	iiod_reader_wait_for_command_done(reader);
	iiod_reader_destroy(reader);
}

static void handle_close(struct parser_pdata *pdata,
			 const struct iiod_command *cmd,
			 struct iiod_command_data *cmd_data,
			 struct iiod_reader *reader)
{
	struct iio_context *ctx = pdata->ctx;
	struct iio_device *dev;
	int ret = -ENXIO;

	dev = iio_context_get_device(ctx, cmd->dev);
	if (!dev)
		goto out_send_response;

	ret = close_dev_helper(pdata, dev);

out_send_response:
	iiod_reader_send_response_code(reader, ret);
}

static const char * get_attr(struct parser_pdata *pdata,
			     const struct iiod_command *cmd)
{
	const struct iio_device *dev;
	const struct iio_channel *chn;
	const char *attr;
	uint16_t arg1 = (uint32_t) cmd->code >> 16,
		 arg2 = cmd->code & 0xffff;

	dev = iio_context_get_device(pdata->ctx, cmd->dev);
	if (!dev)
		return NULL;

	switch (cmd->op) {
	case IIOD_OP_READ_ATTR:
		return iio_device_get_attr(dev, arg1);
	case IIOD_OP_READ_DBG_ATTR:
		return iio_device_get_debug_attr(dev, arg1);
	case IIOD_OP_READ_BUF_ATTR:
		return iio_device_get_buffer_attr(dev, arg1);
	case IIOD_OP_READ_CHN_ATTR:
		chn = iio_device_get_channel(dev, arg2);
		if (!chn)
			break;

		return iio_channel_get_attr(chn, arg1);
	default:
		break;
	}

	return NULL;
}

static ssize_t attr_read(struct parser_pdata *pdata,
			 const struct iiod_command *cmd,
			 const char *attr, void *buf, size_t len)
{
	const struct iio_channel *chn;
	const struct iio_device *dev;
	uint16_t arg2 = cmd->code & 0xffff;

	dev = iio_context_get_device(pdata->ctx, cmd->dev);

	switch (cmd->op) {
	case IIOD_OP_READ_ATTR:
		return iio_device_attr_read(dev, attr, buf, len);
	case IIOD_OP_READ_DBG_ATTR:
		return iio_device_debug_attr_read(dev, attr, buf, len);
	case IIOD_OP_READ_BUF_ATTR:
		return iio_device_buffer_attr_read(dev, attr, buf, len);
	case IIOD_OP_READ_CHN_ATTR:
		chn = iio_device_get_channel(dev, arg2);
		return iio_channel_attr_read(chn, attr, buf, len);
	default:
		return -EINVAL;
	}
}

static ssize_t attr_write(struct parser_pdata *pdata,
			  const struct iiod_command *cmd,
			  const char *attr, const void *buf, size_t len)
{
	const struct iio_channel *chn;
	const struct iio_device *dev;
	uint16_t arg2 = cmd->code & 0xffff;

	dev = iio_context_get_device(pdata->ctx, cmd->dev);

	switch (cmd->op) {
	case IIOD_OP_READ_ATTR:
		return iio_device_attr_write_raw(dev, attr, buf, len);
	case IIOD_OP_READ_DBG_ATTR:
		return iio_device_debug_attr_write_raw(dev, attr, buf, len);
	case IIOD_OP_READ_BUF_ATTR:
		return iio_device_buffer_attr_write_raw(dev, attr, buf, len);
	case IIOD_OP_READ_CHN_ATTR:
		chn = iio_device_get_channel(dev, arg2);
		return iio_channel_attr_write_raw(chn, attr, buf, len);
	default:
		return -EINVAL;
	}
}

static void handle_read_attr(struct parser_pdata *pdata,
			     const struct iiod_command *cmd,
			     struct iiod_command_data *cmd_data,
			     struct iiod_reader *reader)
{
	ssize_t ret = -EINVAL;
	char buf[0x10000];
	const char *attr;
	struct iiod_buf iiod_buf;

	attr = get_attr(pdata, cmd);
	if (attr)
		ret = attr_read(pdata, cmd, attr, buf, sizeof(buf));

	if (ret < 0) {
		iiod_reader_send_response_code(reader, ret);
	} else {
		iiod_buf.ptr = buf;
		iiod_buf.size = ret;

		/* TODO: async? */
		iiod_reader_send_response(reader, ret, &iiod_buf, 1);
	}
}

static void handle_write_attr(struct parser_pdata *pdata,
			      const struct iiod_command *cmd,
			      struct iiod_command_data *cmd_data,
			      struct iiod_reader *reader)
{
	const char *attr;
	size_t count;
	ssize_t ret = -EINVAL;
	uint32_t len;
	struct iiod_buf buf;

	attr = get_attr(pdata, cmd);
	if (!attr)
		goto out_send_response;

	buf.ptr = malloc(cmd->code);
	if (!buf.ptr) {
		ret = -ENOMEM;
		goto out_send_response;
	}

	buf.size = cmd->code;

	ret = iiod_command_data_read(cmd_data, &buf);
	if (ret < 0)
		goto out_free_buf;

	ret = attr_write(pdata, cmd, attr, buf.ptr, cmd->code);

out_free_buf:
	free(buf.ptr);
out_send_response:
	iiod_reader_send_response_code(reader, ret);
}

static void handle_gettrig(struct parser_pdata *pdata,
			   const struct iiod_command *cmd,
			   struct iiod_command_data *cmd_data,
			   struct iiod_reader *reader)
{
	const struct iio_context *ctx = pdata->ctx;
	const struct iio_device *dev, *trigger;
	unsigned int i;
	int ret = -EINVAL;

	dev = iio_context_get_device(ctx, cmd->dev);
	if (!dev)
		goto out_send_response;

	ret = iio_device_get_trigger(dev, &trigger);
	if (!ret && !trigger)
		ret = -ENODEV;
	if (ret)
		goto out_send_response;

	for (i = 0; i < iio_context_get_devices_count(ctx); i++)
		if (trigger == iio_context_get_device(ctx, i))
			break;

	ret = i < iio_context_get_devices_count(ctx) ? i : -ENODEV;

out_send_response:
	iiod_reader_send_response_code(reader, ret);
}

static void handle_settrig(struct parser_pdata *pdata,
			   const struct iiod_command *cmd,
			   struct iiod_command_data *cmd_data,
			   struct iiod_reader *reader)
{
	const struct iio_context *ctx = pdata->ctx;
	const struct iio_device *dev, *trigger;
	int ret = -EINVAL;

	dev = iio_context_get_device(ctx, cmd->dev);
	if (!dev)
		goto out_send_response;

	if (cmd->code == -1) {
		trigger = NULL;
	} else {
		trigger = iio_context_get_device(ctx, cmd->code);
		if (!trigger)
			goto out_send_response;
	}

	ret = iio_device_set_trigger(dev, trigger);

out_send_response:
	iiod_reader_send_response_code(reader, ret);
}

static void handle_setbufcnt(struct parser_pdata *pdata,
			     const struct iiod_command *cmd,
			     struct iiod_command_data *cmd_data,
			     struct iiod_reader *reader)
{
	const struct iio_device *dev;
	int ret = -EINVAL;

	dev = iio_context_get_device(pdata->ctx, cmd->dev);
	if (!dev)
		goto out_send_response;

	ret = iio_device_set_kernel_buffers_count(dev, cmd->code);

out_send_response:
	iiod_reader_send_response_code(reader, ret);
}

typedef void (*iiod_opcode_fn)(struct parser_pdata *,
			       const struct iiod_command *,
			       struct iiod_command_data *cmd_data,
			       struct iiod_reader *);

static const iiod_opcode_fn iiod_op_functions[] = {
	[IIOD_OP_PRINT]			= handle_print,
	[IIOD_OP_TIMEOUT]		= handle_timeout,
	[IIOD_OP_CLOSE]			= handle_close,
	[IIOD_OP_READ_ATTR]		= handle_read_attr,
	[IIOD_OP_READ_DBG_ATTR]		= handle_read_attr,
	[IIOD_OP_READ_BUF_ATTR]		= handle_read_attr,
	[IIOD_OP_READ_CHN_ATTR]		= handle_read_attr,
	[IIOD_OP_WRITE_ATTR]		= handle_write_attr,
	[IIOD_OP_WRITE_DBG_ATTR]	= handle_write_attr,
	[IIOD_OP_WRITE_BUF_ATTR]	= handle_write_attr,
	[IIOD_OP_WRITE_CHN_ATTR]	= handle_write_attr,
	[IIOD_OP_GETTRIG]		= handle_gettrig,
	[IIOD_OP_SETTRIG]		= handle_settrig,
	[IIOD_OP_SETBUFCNT]		= handle_setbufcnt,
};

static int iiod_cmd(const struct iiod_command *cmd,
		    struct iiod_command_data *data, void *d)
{
	struct parser_pdata *pdata = d;
	struct iiod_reader *reader;

	if (cmd->op >= IIOD_NB_OPCODES) {
		IIO_ERROR("Received invalid opcode 0x%x\n", cmd->op);
		return -EINVAL;
	}

	if (cmd->op == IIOD_OP_OPEN || cmd->op == IIOD_OP_OPEN_CYCLIC) {
		handle_open(d, cmd, data);

		return 0;
	}

	reader = iiod_command_create_reader(cmd, data);
	if (!reader)
		return -ENOMEM;

	iiod_op_functions[cmd->op](d, cmd, data, reader);

	printf("Function done.\n");
	iiod_reader_destroy(reader);
	printf("Reader destroyed\n");

	return 0;
}

static ssize_t iiod_read(void *d, const struct iiod_buf *buf, size_t nb)
{
	return read_all(d, buf->ptr, buf->size);
}

static ssize_t iiod_write(void *d, const struct iiod_buf *buf, size_t nb)
{
	return write_all(d, buf->ptr, buf->size);
}

static const struct iiod_responder_ops iiod_responder_ops = {
	.cmd	= iiod_cmd,
	.read	= iiod_read,
	.write	= iiod_write,
};

int binary_parse(struct parser_pdata *pdata)
{
	struct iiod_responder *responder;
	struct iiod_command cmd;
	uint32_t response;
	ssize_t ret;

	responder = iiod_responder_create(&iiod_responder_ops, pdata);
	if (!responder)
		return -ENOMEM;

	/* TODO: poll main thread pool FD */

	iiod_responder_wait_done(responder);
	iiod_responder_destroy(responder);

	return 0;
}
