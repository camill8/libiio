// SPDX-License-Identifier: LGPL-2.1-or-later
/*
 * libiio - Library for interfacing industrial I/O (IIO) devices
 *
 * Copyright (C) 2014-2020 Analog Devices, Inc.
 * Author: Paul Cercueil <paul.cercueil@analog.com>
 */

#include "iiod-client.h"
#include "iiod-responder.h"
#include "iio-config.h"
#include "iio-debug.h"
#include "iio-lock.h"
#include "iio-private.h"

#include <errno.h>
#include <inttypes.h>
#include <string.h>
#include <stdio.h>
#if WITH_ZSTD
#include <zstd.h>
#endif

struct iiod_client {
	const struct iio_context_params *params;
	struct iiod_client_pdata *desc;
	const struct iiod_client_ops *ops;
	struct iio_mutex *lock;

	struct iiod_responder *responder;

	/* Reader for synchronous I/O */
	struct iiod_reader *reader;
};

struct iiod_client_io {
	struct iiod_client *client;
	bool cyclic;
	size_t samples_count;
	size_t buffer_size;

	const struct iio_device *dev;
};

static int iiod_client_enable_binary(struct iiod_client *client);

void iiod_client_mutex_lock(struct iiod_client *client)
{
	iio_mutex_lock(client->lock);
}

void iiod_client_mutex_unlock(struct iiod_client *client)
{
	iio_mutex_unlock(client->lock);
}

static ssize_t iiod_client_read_integer(struct iiod_client *client, int *val)
{
	unsigned int i;
	char buf[1024], *ptr = NULL, *end;
	ssize_t ret;
	int value;

	do {
		ret = client->ops->read_line(client->desc, buf, sizeof(buf));
		if (ret < 0) {
			prm_err(client->params, "READ LINE: %zd\n", ret);
			return ret;
		}

		for (i = 0; i < (unsigned int) ret; i++) {
			if (buf[i] != '\n') {
				if (!ptr)
					ptr = &buf[i];
			} else if (!!ptr) {
				break;
			}
		}
	} while (!ptr);

	buf[i] = '\0';

	errno = 0;
	value = (int) strtol(ptr, &end, 10);
	if (ptr == end || errno == ERANGE)
		return -EINVAL;

	*val = value;
	return 0;
}

static int iiod_client_exec_command(struct iiod_client *client, const char *cmd)
{
	int resp;
	ssize_t ret;

	ret = client->ops->write(client->desc, cmd, strlen(cmd));
	if (ret < 0)
		return (int) ret;

	ret = iiod_client_read_integer(client, &resp);
	return ret < 0 ? (int) ret : resp;
}

static ssize_t iiod_client_write_all(struct iiod_client *client,
				     const void *src, size_t len)
{
	const struct iiod_client_ops *ops = client->ops;
	struct iiod_client_pdata *desc = client->desc;
	uintptr_t ptr = (uintptr_t) src;

	while (len) {
		ssize_t ret = ops->write(desc, (const void *) ptr, len);

		if (ret < 0) {
			if (ret == -EINTR)
				continue;
			else
				return ret;
		}

		if (ret == 0)
			return -EPIPE;

		ptr += ret;
		len -= ret;
	}

	return (ssize_t) (ptr - (uintptr_t) src);
}

static ssize_t iiod_client_read_all(struct iiod_client *client,
				    void *dst, size_t len)
{
	const struct iiod_client_ops *ops = client->ops;
	uintptr_t ptr = (uintptr_t) dst;

	while (len) {
		ssize_t ret = ops->read(client->desc, (void *) ptr, len);

		if (ret < 0) {
			if (ret == -EINTR)
				continue;
			else
				return ret;
		}

		if (ret == 0)
			return -EPIPE;

		ptr += ret;
		len -= ret;
	}

	return (ssize_t) (ptr - (uintptr_t) dst);
}

struct iiod_client * iiod_client_new(const struct iio_context_params *params,
				     struct iiod_client_pdata *desc,
				     const struct iiod_client_ops *ops)
{
	struct iiod_client *client;

	client = malloc(sizeof(*client));
	if (!client) {
		errno = ENOMEM;
		return NULL;
	}

	client->lock = iio_mutex_create();
	if (!client->lock) {
		errno = ENOMEM;
		goto err_free_client;
	}

	client->params = params;
	client->ops = ops;
	client->desc = desc;
	client->responder = NULL;
	client->reader = NULL;

	iiod_client_enable_binary(client);

	return client;

err_free_client:
	free(client);
	return NULL;
}

void iiod_client_destroy(struct iiod_client *client)
{
	if (client->reader)
		iiod_reader_destroy(client->reader);
	if (client->responder)
		iiod_responder_destroy(client->responder);

	iio_mutex_destroy(client->lock);
	free(client);
}

static int iio_device_get_index(const struct iio_device *dev)
{
	const struct iio_context *ctx = iio_device_get_context(dev);
	unsigned int idx;

	for (idx = 0; idx < iio_context_get_devices_count(ctx); idx++)
		if (dev == iio_context_get_device(ctx, idx))
			return idx;

	return -ENODEV; /* Cannot happen */
}

static int iiod_client_get_trigger_new(struct iiod_client *client,
				       const struct iio_device *dev,
				       const struct iio_device **trigger)
{
	const struct iio_context *ctx = iio_device_get_context(dev);
	struct iiod_command cmd;
	int ret;

	cmd.op = IIOD_OP_GETTRIG;
	cmd.dev = iio_device_get_index(dev);

	ret = iiod_reader_exec_simple_command(client->reader, &cmd);
	if (ret == -ENODEV)
		*trigger = NULL;
	else if (ret >= 0)
		*trigger = iio_context_get_device(ctx, ret);
	else
		return ret;

	return 0;
}

int iiod_client_get_trigger(struct iiod_client *client,
			    const struct iio_device *dev,
			    const struct iio_device **trigger)
{
	const struct iio_context *ctx = iio_device_get_context(dev);
	unsigned int i, nb_devices = iio_context_get_devices_count(ctx);
	char buf[1024];
	unsigned int name_len;
	int ret;

	if (client->responder)
		return iiod_client_get_trigger_new(client, dev, trigger);

	iio_snprintf(buf, sizeof(buf), "GETTRIG %s\r\n",
			iio_device_get_id(dev));

	iio_mutex_lock(client->lock);
	ret = iiod_client_exec_command(client, buf);

	if (ret == 0)
		*trigger = NULL;
	if (ret <= 0)
		goto out_unlock;

	if ((unsigned int) ret > sizeof(buf) - 1) {
		ret = -EIO;
		goto out_unlock;
	}

	name_len = ret;

	ret = (int) iiod_client_read_all(client, buf, name_len + 1);
	if (ret < 0)
		goto out_unlock;

	ret = -ENXIO;

	for (i = 0; i < nb_devices; i++) {
		struct iio_device *cur = iio_context_get_device(ctx, i);

		if (iio_device_is_trigger(cur)) {
			const char *name = iio_device_get_name(cur);

			if (!name)
				continue;

			if (!strncmp(name, buf, name_len)) {
				*trigger = cur;
				ret = 0;
				goto out_unlock;
			}
		}
	}

out_unlock:
	iio_mutex_unlock(client->lock);
	return ret;
}

static int iiod_client_set_trigger_new(struct iiod_client *client,
				       const struct iio_device *dev,
				       const struct iio_device *trigger)
{
	struct iiod_command cmd;

	cmd.op = IIOD_OP_SETTRIG;
	cmd.dev = iio_device_get_index(dev);
	if (!trigger)
		cmd.code = -1;
	else
		cmd.code = iio_device_get_index(trigger);

	return iiod_reader_exec_simple_command(client->reader, &cmd);
}

int iiod_client_set_trigger(struct iiod_client *client,
			    const struct iio_device *dev,
			    const struct iio_device *trigger)
{
	char buf[1024];
	int ret;

	if (client->responder)
		return iiod_client_set_trigger_new(client, dev, trigger);

	if (trigger) {
		iio_snprintf(buf, sizeof(buf), "SETTRIG %s %s\r\n",
				iio_device_get_id(dev),
				iio_device_get_id(trigger));
	} else {
		iio_snprintf(buf, sizeof(buf), "SETTRIG %s\r\n",
				iio_device_get_id(dev));
	}

	iio_mutex_lock(client->lock);
	ret = iiod_client_exec_command(client, buf);
	iio_mutex_unlock(client->lock);
	return ret;
}

static int
iiod_client_set_kernel_buffers_count_new(struct iiod_client *client,
					 const struct iio_device *dev,
					 unsigned int nb_blocks)
{
	struct iiod_command cmd;

	cmd.op = IIOD_OP_SETBUFCNT;
	cmd.dev = iio_device_get_index(dev);
	cmd.code = nb_blocks;

	return iiod_reader_exec_simple_command(client->reader, &cmd);
}

int iiod_client_set_kernel_buffers_count(struct iiod_client *client,
					 const struct iio_device *dev,
					 unsigned int nb_blocks)
{
	int ret;
	char buf[1024];

	if (client->responder) {
		return iiod_client_set_kernel_buffers_count_new(client,
								dev, nb_blocks);
	}

	iio_snprintf(buf, sizeof(buf), "SET %s BUFFERS_COUNT %u\r\n",
			iio_device_get_id(dev), nb_blocks);

	iio_mutex_lock(client->lock);
	ret = iiod_client_exec_command(client, buf);
	iio_mutex_unlock(client->lock);
	return ret;
}

int iiod_client_set_timeout(struct iiod_client *client, unsigned int timeout)
{
	int ret;

	if (client->responder) {
		struct iiod_command cmd;

		cmd.op = IIOD_OP_TIMEOUT;
		cmd.code = timeout;

		ret = iiod_reader_exec_simple_command(client->reader, &cmd);
	} else {
		char buf[1024];

		iio_mutex_lock(client->lock);
		iio_snprintf(buf, sizeof(buf), "TIMEOUT %u\r\n", timeout);
		ret = iiod_client_exec_command(client, buf);
		iio_mutex_unlock(client->lock);
	}

	return ret;
}

static int iiod_client_discard(struct iiod_client *client,
			       char *buf, size_t buf_len, size_t to_discard)
{
	do {
		size_t read_len;
		ssize_t ret;

		if (to_discard > buf_len)
			read_len = buf_len;
		else
			read_len = to_discard;

		ret = iiod_client_read_all(client, buf, read_len);
		if (ret < 0)
			return (int) ret;

		to_discard -= (size_t) ret;
	} while (to_discard);

	return 0;
}

static ssize_t iiod_client_read_attr_new(struct iiod_client *client,
					 const struct iio_device *dev,
					 const struct iio_channel *chn,
					 const char *attr, char *dest,
					 size_t len, enum iio_attr_type type)
{
	struct iiod_command cmd = { 0 };
	unsigned int i;
	uint16_t arg1, arg2 = 0;
	struct iiod_buf iiod_buf;

	if (chn) {
		cmd.op = IIOD_OP_READ_CHN_ATTR;

		for (i = 0; i < iio_device_get_channels_count(dev); i++)
			if (iio_device_get_channel(dev, i) == chn)
				break;

		arg2 = i;

		for (i = 0; i < iio_channel_get_attrs_count(chn); i++)
			if (!strcmp(iio_channel_get_attr(chn, i), attr))
				break;

		arg1 = i;
	} else {
		switch (type) {
		default:
			cmd.op = IIOD_OP_READ_ATTR;

			for (i = 0; i < iio_device_get_attrs_count(dev); i++)
				if (!strcmp(iio_device_get_attr(dev, i), attr))
					break;

			arg1 = i;
			break;
		case IIO_ATTR_TYPE_DEBUG:
			cmd.op = IIOD_OP_READ_DBG_ATTR;

			for (i = 0; i < iio_device_get_debug_attrs_count(dev); i++)
				if (!strcmp(iio_device_get_debug_attr(dev, i), attr))
					break;

			arg1 = i;
			break;
		case IIO_ATTR_TYPE_BUFFER:
			cmd.op = IIOD_OP_READ_BUF_ATTR;

			for (i = 0; i < iio_device_get_buffer_attrs_count(dev); i++)
				if (!strcmp(iio_device_get_buffer_attr(dev, i), attr))
					break;

			arg1 = i;
			break;
		}
	}

	cmd.dev = iio_device_get_index(dev);
	cmd.code = (arg1 << 16) | arg2;

	iiod_buf.ptr = dest;
	iiod_buf.size = len;

	return iiod_reader_exec_command(client->reader, &cmd, NULL, &iiod_buf);
}

ssize_t iiod_client_read_attr(struct iiod_client *client,
			      const struct iio_device *dev,
			      const struct iio_channel *chn,
			      const char *attr, char *dest,
			      size_t len, enum iio_attr_type type)
{
	const char *id = iio_device_get_id(dev);
	char buf[1024];
	ssize_t ret;

	if (client->responder) {
		return iiod_client_read_attr_new(client, dev, chn,
						 attr, dest, len, type);
	}

	if (attr) {
		if (chn) {
			if (!iio_channel_find_attr(chn, attr))
				return -ENOENT;
		} else {
			switch (type) {
				case IIO_ATTR_TYPE_DEVICE:
					if (!iio_device_find_attr(dev, attr))
						return -ENOENT;
					break;
				case IIO_ATTR_TYPE_DEBUG:
					if (!iio_device_find_debug_attr(dev, attr))
						return -ENOENT;
					break;
				case IIO_ATTR_TYPE_BUFFER:
					if (!iio_device_find_buffer_attr(dev, attr))
						return -ENOENT;
					break;
				default:
					return -EINVAL;
			}
		}
	}

	if (chn) {
		iio_snprintf(buf, sizeof(buf), "READ %s %s %s %s\r\n", id,
				iio_channel_is_output(chn) ? "OUTPUT" : "INPUT",
				iio_channel_get_id(chn), attr ? attr : "");
	} else {
		switch (type) {
			case IIO_ATTR_TYPE_DEVICE:
				iio_snprintf(buf, sizeof(buf), "READ %s %s\r\n",
						id, attr ? attr : "");
				break;
			case IIO_ATTR_TYPE_DEBUG:
				iio_snprintf(buf, sizeof(buf), "READ %s DEBUG %s\r\n",
						id, attr ? attr : "");
				break;
			case IIO_ATTR_TYPE_BUFFER:
				iio_snprintf(buf, sizeof(buf), "READ %s BUFFER %s\r\n",
						id, attr ? attr : "");
				break;
		}
	}

	iio_mutex_lock(client->lock);

	ret = (ssize_t) iiod_client_exec_command(client, buf);
	if (ret < 0)
		goto out_unlock;

	if ((size_t) ret + 1 > len) {
		iiod_client_discard(client, dest, len, ret + 1);
		ret = -EIO;
		goto out_unlock;
	}

	/* +1: Also read the trailing \n */
	ret = iiod_client_read_all(client, dest, ret + 1);

	if (ret > 0) {
		/* Discard the trailing \n */
		ret--;

		/* Replace it with a \0 just in case */
		dest[ret] = '\0';
	}

out_unlock:
	iio_mutex_unlock(client->lock);
	return ret;
}

ssize_t iiod_client_write_attr(struct iiod_client *client,
			       const struct iio_device *dev,
			       const struct iio_channel *chn,
			       const char *attr, const char *src,
			       size_t len, enum iio_attr_type type)
{
	const struct iiod_client_ops *ops = client->ops;
	const char *id = iio_device_get_id(dev);
	char buf[1024];
	ssize_t ret;
	int resp;

	if (attr) {
		if (chn) {
			if (!iio_channel_find_attr(chn, attr))
				return -ENOENT;
		} else {
			switch (type) {
				case IIO_ATTR_TYPE_DEVICE:
					if (!iio_device_find_attr(dev, attr))
						return -ENOENT;
					break;
				case IIO_ATTR_TYPE_DEBUG:
					if (!iio_device_find_debug_attr(dev, attr))
						return -ENOENT;
					break;
				case IIO_ATTR_TYPE_BUFFER:
					if (!iio_device_find_buffer_attr(dev, attr))
						return -ENOENT;
					break;
				default:
					return -EINVAL;
			}
		}
	}

	if (chn) {
		iio_snprintf(buf, sizeof(buf), "WRITE %s %s %s %s %lu\r\n", id,
				iio_channel_is_output(chn) ? "OUTPUT" : "INPUT",
				iio_channel_get_id(chn), attr ? attr : "",
				(unsigned long) len);
	} else {
		switch (type) {
			case IIO_ATTR_TYPE_DEVICE:
				iio_snprintf(buf, sizeof(buf), "WRITE %s %s %lu\r\n",
						id, attr ? attr : "", (unsigned long) len);
				break;
			case IIO_ATTR_TYPE_DEBUG:
				iio_snprintf(buf, sizeof(buf), "WRITE %s DEBUG %s %lu\r\n",
						id, attr ? attr : "", (unsigned long) len);
				break;
			case IIO_ATTR_TYPE_BUFFER:
				iio_snprintf(buf, sizeof(buf), "WRITE %s BUFFER %s %lu\r\n",
						id, attr ? attr : "", (unsigned long) len);
				break;
		}
	}

	iio_mutex_lock(client->lock);
	ret = ops->write(client->desc, buf, strlen(buf));
	if (ret < 0)
		goto out_unlock;

	ret = iiod_client_write_all(client, src, len);
	if (ret < 0)
		goto out_unlock;

	ret = iiod_client_read_integer(client, &resp);
	if (ret < 0)
		goto out_unlock;

	ret = (ssize_t) resp;

out_unlock:
	iio_mutex_unlock(client->lock);
	return ret;
}

static int iiod_client_cmd(const struct iiod_command *cmd,
			   struct iiod_command_data *data, void *d)
{
	/* We don't support receiving commands. */

	return -EINVAL;
}

static ssize_t
iiod_client_read_cb(void *d, const struct iiod_buf *buf, size_t nb)
{
	struct iiod_client *client = d;
	ssize_t ret, count = 0;
	unsigned int i;

	for (i = 0; i < nb; i++) {
		ret = iiod_client_read_all(client, buf[i].ptr, buf[i].size);
		if (ret <= 0)
			return ret;

		count += ret;
	}

	return count;
}

static ssize_t
iiod_client_write_cb(void *d, const struct iiod_buf *buf, size_t nb)
{
	struct iiod_client *client = d;
	ssize_t ret, count = 0;
	unsigned int i;

	for (i = 0; i < nb; i++) {
		return iiod_client_write_all(client, buf[i].ptr, buf[i].size);
		if (ret <= 0)
			return ret;

		count += ret;
	}

	return count;
}

static ssize_t iiod_client_discard_cb(void *d, size_t bytes)
{
	struct iiod_client *client = d;
	char buf[0x1000];
	int ret;

	ret = iiod_client_discard(client, buf, sizeof(buf), bytes);
	if (ret < 0)
		return ret;

	return bytes;
}

static const struct iiod_responder_ops iiod_client_ops = {
	.cmd		= iiod_client_cmd,
	.read		= iiod_client_read_cb,
	.write		= iiod_client_write_cb,
	.discard	= iiod_client_discard_cb,
};

static int iiod_client_enable_binary(struct iiod_client *client)
{
	int ret;

	ret = iiod_client_exec_command(client, "BINARY\r\n");

	/* If the BINARY command fail, don't create the responder */
	if (ret != 0)
		return 0;

	client->responder = iiod_responder_create(&iiod_client_ops, client);
	if (!client->responder) {
		prm_err(client->params, "Unable to create responder\n");
		return -ENOMEM;
	}

	client->reader = iiod_responder_create_reader(client->responder);
	if (!client->reader) {
		prm_err(client->params, "Unable to create reader\n");
		iiod_responder_destroy(client->responder);
		client->responder = NULL;
		return -ENOMEM;
	}

	return 0;
}

static int iiod_client_send_print(struct iiod_client *client,
				  void *buf, size_t buf_len)
{
	struct iiod_command cmd = { .op = IIOD_OP_PRINT };
	struct iiod_buf iiod_buf = { .ptr = buf, .size = buf_len };

	return iiod_reader_exec_command(client->reader, &cmd, NULL, &iiod_buf);
}

static struct iio_context *
iiod_client_create_context_private_new(struct iiod_client *client,
				       const struct iio_backend *backend,
				       const char *description,
				       const char **ctx_attrs,
				       const char **ctx_values,
				       unsigned int nb_ctx_attrs)
{
	struct iio_context *ctx = NULL;
	unsigned long long len;
	size_t xml_len = 0x10000;
	char *xml_zstd, *xml;
	int ret;

	xml = malloc(xml_len);
	if (!xml)
		return NULL;

	ret = iiod_client_send_print(client, xml, xml_len);
	if (ret < 0) {
		prm_perror(client->params, -ret,
			   "Unable to send PRINT command");
		goto out_free_xml;
	}

	xml_len = ret;


	prm_dbg(client->params, "Received ZSTD-compressed XML string.\n");

	len = ZSTD_getFrameContentSize(xml, xml_len);
	if (len == ZSTD_CONTENTSIZE_UNKNOWN ||
	    len == ZSTD_CONTENTSIZE_ERROR) {
		ret = -EIO;
		goto out_free_xml;
	}

	xml_zstd = malloc(len);
	if (!xml_zstd) {
		ret = -ENOMEM;
		goto out_free_xml;
	}

	xml_len = ZSTD_decompress(xml_zstd, len, xml, xml_len);
	if (ZSTD_isError(xml_len)) {
		prm_err(client->params, "Unable to decompress ZSTD data: %s\n",
			ZSTD_getErrorName(xml_len));
		ret = -EIO;
		free(xml_zstd);
		goto out_free_xml;
	}

	/* Free compressed data, make "xml" point to uncompressed data */
	free(xml);
	xml = xml_zstd;

	prm_dbg(client->params, "Creating context\n");

	ctx = iio_create_context_from_xml(client->params, xml, xml_len,
					  backend, description,
					  ctx_attrs, ctx_values, nb_ctx_attrs);
	if (!ctx) {
		ret = -errno;
	} else {
		/* If the context creation suceeded, update our "params" pointer
		 * to point to the context's params, as we know their lifetime
		 * is bigger than ours. */
		client->params = &ctx->params;
	}

	if (ctx)
		prm_dbg(client->params, "Context created.\n");

out_free_xml:
	free(xml);
	if (!ctx)
		errno = -ret;
	return ctx;
}

static struct iio_context *
iiod_client_create_context_private(struct iiod_client *client,
				   const struct iio_backend *backend,
				   const char *description,
				   const char **ctx_attrs,
				   const char **ctx_values,
				   unsigned int nb_ctx_attrs,
				   bool zstd)
{
	const char *cmd = zstd ? "ZPRINT\r\n" : "PRINT\r\n";
	struct iio_context *ctx = NULL;
	unsigned int extra_char = !client->responder;
	size_t xml_len;
	char *xml;
	int ret;

	iio_mutex_lock(client->lock);
	ret = iiod_client_exec_command(client, cmd);
	if (ret == -EINVAL && zstd) {
		/* If the ZPRINT command does not exist, try again
		 * with the regular PRINT command. */
		iio_mutex_unlock(client->lock);

		return iiod_client_create_context_private(client,
							  backend, description,
							  ctx_attrs, ctx_values,
							  nb_ctx_attrs, false);
	}
	if (ret < 0)
		goto out_unlock;

	xml_len = (size_t) ret;
	xml = malloc(xml_len + extra_char);
	if (!xml) {
		ret = -ENOMEM;
		goto out_unlock;
	}

	ret = (int) iiod_client_read_all(client, xml, xml_len + extra_char);
	if (ret < 0)
		goto out_free_xml;

#if WITH_ZSTD
	if (zstd) {
		unsigned long long len;
		char *xml_zstd;

		prm_dbg(client->params, "Received ZSTD-compressed XML string.\n");

		len = ZSTD_getFrameContentSize(xml, xml_len);
		if (len == ZSTD_CONTENTSIZE_UNKNOWN ||
		    len == ZSTD_CONTENTSIZE_ERROR) {
			ret = -EIO;
			goto out_free_xml;
		}

		xml_zstd = malloc(len);
		if (!xml_zstd) {
			ret = -ENOMEM;
			goto out_free_xml;
		}

		xml_len = ZSTD_decompress(xml_zstd, len, xml, xml_len);
		if (ZSTD_isError(xml_len)) {
			prm_err(client->params, "Unable to decompress ZSTD data: %s\n",
				ZSTD_getErrorName(xml_len));
			ret = -EIO;
			free(xml_zstd);
			goto out_free_xml;
		}

		/* Free compressed data, make "xml" point to uncompressed data */
		free(xml);
		xml = xml_zstd;
	}
#endif

	ctx = iio_create_context_from_xml(client->params, xml, xml_len,
					  backend, description,
					  ctx_attrs, ctx_values, nb_ctx_attrs);
	if (!ctx) {
		ret = -errno;
	} else {
		/* If the context creation suceeded, update our "params" pointer
		 * to point to the context's params, as we know their lifetime
		 * is bigger than ours. */
		client->params = &ctx->params;
	}

out_free_xml:
	free(xml);
out_unlock:
	iio_mutex_unlock(client->lock);
	if (!ctx)
		errno = -ret;
	return ctx;
}

struct iio_context * iiod_client_create_context(struct iiod_client *client,
						const struct iio_backend *backend,
						const char *description,
						const char **ctx_attrs,
						const char **ctx_values,
						unsigned int nb_ctx_attrs)
{
	if (!WITH_ZSTD || !client->responder)
		return iiod_client_create_context_private(client, backend,
							  description,
							  ctx_attrs, ctx_values,
							  nb_ctx_attrs,
							  WITH_ZSTD);


	return iiod_client_create_context_private_new(client, backend,
						      description, ctx_attrs,
						      ctx_values, nb_ctx_attrs);
}

static struct iiod_client_io *
iiod_client_create_io_context(struct iiod_client *client,
			      const struct iio_device *dev,
			      size_t samples_count, bool cyclic)
{
	struct iiod_client_io *io;

	io = zalloc(sizeof(*io));
	if (!io)
		return NULL;

	io->client = client;
	io->dev = dev;
	io->samples_count = samples_count;
	io->cyclic = cyclic;

	return io;
}

static void iiod_client_io_destroy(struct iiod_client_io *io)
{
	free(io);
}

struct iiod_client_io *
iiod_client_open_unlocked(struct iiod_client *client,
			  const struct iio_device *dev,
			  size_t samples_count, bool cyclic)
{
	struct iiod_client_io *io;
	char buf[1024], *ptr;
	size_t i;
	ssize_t len;
	int ret;

	io = iiod_client_create_io_context(client, dev, samples_count, cyclic);
	if (!io)
		return ERR_PTR(-ENOMEM);

	if (client->responder)
		return ERR_PTR(-ENOSYS); /* TODO */

	len = sizeof(buf);
	len -= iio_snprintf(buf, len, "OPEN %s %lu ",
			iio_device_get_id(dev), (unsigned long) samples_count);
	ptr = buf + strlen(buf);

	for (i = dev->words; i > 0; i--, ptr += 8) {
		len -= iio_snprintf(ptr, len, "%08" PRIx32,
				dev->mask[i - 1]);
	}

	len -= iio_strlcpy(ptr, cyclic ? " CYCLIC\r\n" : "\r\n", len);

	if (len < 0) {
		prm_err(client->params, "strlength problem in iiod_client_open_unlocked\n");
		ret = -ENOMEM;
		goto err_destroy_io;
	}

	ret = iiod_client_exec_command(client, buf);
	if (ret < 0)
		goto err_destroy_io;

	return io;

err_destroy_io:
	iiod_client_io_destroy(io);

	return ERR_PTR(ret);
}

int iiod_client_close_unlocked(struct iiod_client_io *io)
{
	struct iiod_command cmd;
	char buf[1024];
	int ret;

	if (io->client->responder) {
		cmd.op = IIOD_OP_CLOSE;
		cmd.dev = iio_device_get_index(io->dev);

		ret = iiod_reader_exec_simple_command(io->client->reader, &cmd);
	} else {
		iio_snprintf(buf, sizeof(buf), "CLOSE %s\r\n",
			     iio_device_get_id(io->dev));
		ret = iiod_client_exec_command(io->client, buf);
	}

	iiod_client_io_destroy(io);

	return ret;
}

static int iiod_client_read_mask(struct iiod_client *client,
				 uint32_t *mask, size_t words)
{
	size_t i;
	ssize_t ret;
	char *buf, *ptr;

	buf = malloc(words * 8 + 1);
	if (!buf)
		return -ENOMEM;

	ret = iiod_client_read_all(client, buf, words * 8 + 1);
	if (ret < 0) {
		prm_err(NULL, "READ ALL: %zd\n", ret);
		goto out_buf_free;
	} else
		ret = 0;

	buf[words*8] = '\0';

	prm_dbg(client->params, "Reading mask\n");

	for (i = words, ptr = buf; i > 0; i--) {
		iio_sscanf(ptr, "%08" PRIx32, &mask[i - 1]);
		prm_dbg(client->params, "mask[%lu] = 0x%08" PRIx32 "\n",
				(unsigned long)(i - 1), mask[i - 1]);

		ptr = (char *) ((uintptr_t) ptr + 8);
	}

out_buf_free:
	free(buf);
	return (int) ret;
}

static ssize_t iiod_client_read_unlocked(struct iiod_client *client,
					 const struct iio_device *dev,
					 void *dst, size_t len,
					 uint32_t *mask, size_t words)
{
	unsigned int nb_channels = iio_device_get_channels_count(dev);
	uintptr_t ptr = (uintptr_t) dst;
	char buf[1024];
	ssize_t ret, read = 0;

	if (client->responder)
		return -ENOSYS;

	if (!len || words != (nb_channels + 31) / 32)
		return -EINVAL;

	iio_snprintf(buf, sizeof(buf), "READBUF %s %lu\r\n",
			iio_device_get_id(dev), (unsigned long) len);

	ret = iiod_client_write_all(client, buf, strlen(buf));
	if (ret < 0) {
		prm_err(client->params, "WRITE ALL: %zd\n", ret);
		return ret;
	}

	do {
		int to_read;

		ret = iiod_client_read_integer(client, &to_read);
		if (ret < 0) {
			prm_err(client->params, "READ INTEGER: %zd\n", ret);
			return ret;
		}
		if (to_read < 0)
			return (ssize_t) to_read;
		if (!to_read)
			break;

		if (mask) {
			ret = iiod_client_read_mask(client, mask, words);
			if (ret < 0) {
				prm_err(client->params, "READ ALL: %zd\n", ret);
				return ret;
			}

			mask = NULL; /* We read the mask only once */
		}

		ret = iiod_client_read_all(client, (char *) ptr, to_read);
		if (ret < 0)
			return ret;

		ptr += ret;
		read += ret;
		len -= ret;
	} while (len);

	return read;
}

ssize_t iiod_client_read(struct iiod_client *client,
			 const struct iio_device *dev,
			 void *dst, size_t len,
			 uint32_t *mask, size_t words)
{
	ssize_t ret;

	iiod_client_mutex_lock(client);
	ret = iiod_client_read_unlocked(client, dev, dst, len, mask, words);
	iiod_client_mutex_unlock(client);

	return ret;
}

static ssize_t iiod_client_write_unlocked(struct iiod_client *client,
					  const struct iio_device *dev,
					  const void *src, size_t len)
{
	ssize_t ret;
	char buf[1024];
	int val;

	if (client->responder)
		return -ENOSYS;

	iio_snprintf(buf, sizeof(buf), "WRITEBUF %s %lu\r\n",
			dev->id, (unsigned long) len);

	ret = iiod_client_write_all(client, buf, strlen(buf));
	if (ret < 0)
		return ret;

	ret = iiod_client_read_integer(client, &val);
	if (ret < 0)
		return ret;
	if (val < 0)
		return (ssize_t) val;

	ret = iiod_client_write_all(client, src, len);
	if (ret < 0)
		return ret;

	ret = iiod_client_read_integer(client, &val);
	if (ret < 0)
		return ret;
	if (val < 0)
		return (ssize_t) val;

	return (ssize_t) len;
}

ssize_t iiod_client_write(struct iiod_client *client,
			  const struct iio_device *dev,
			  const void *src, size_t len)
{
	ssize_t ret;

	iiod_client_mutex_lock(client);
	ret = iiod_client_write_unlocked(client, dev, src, len);
	iiod_client_mutex_unlock(client);

	return ret;
}
