/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * libiio - Library for interfacing industrial I/O (IIO) devices
 *
 * Copyright (C) 2021 Analog Devices, Inc.
 * Author: Paul Cercueil <paul.cercueil@analog.com>
 */

#ifndef __IIOD_RESPONDER_H__
#define __IIOD_RESPONDER_H__

#include <stddef.h>

struct iiod_command_data;
struct iiod_responder;
struct iiod_reader;

enum iiod_opcode {
	IIOD_OP_RESPONSE,
	IIOD_OP_PRINT,
	IIOD_OP_VERSION,
	IIOD_OP_TIMEOUT,
	IIOD_OP_OPEN,
	IIOD_OP_OPEN_CYCLIC,
	IIOD_OP_CLOSE,
	IIOD_OP_READ_ATTR,
	IIOD_OP_READ_DBG_ATTR,
	IIOD_OP_READ_BUF_ATTR,
	IIOD_OP_READ_CHN_ATTR,
	IIOD_OP_WRITE_ATTR,
	IIOD_OP_WRITE_DBG_ATTR,
	IIOD_OP_WRITE_BUF_ATTR,
	IIOD_OP_WRITE_CHN_ATTR,
	IIOD_OP_READBUF,
	IIOD_OP_WRITEBUF,
	IIOD_OP_GETTRIG,
	IIOD_OP_SETTRIG,
	IIOD_OP_SETBUFCNT,

	IIOD_NB_OPCODES,
};

struct iiod_command {
	uint16_t client_id;
	uint8_t op;
	uint8_t dev;
	int32_t code;
};

struct iiod_buf {
	void *ptr;
	size_t size;
};

struct iiod_responder_ops {
	int (*cmd)(const struct iiod_command *cmd,
		   struct iiod_command_data *data, void *d);
	ssize_t (*read)(void *d, const struct iiod_buf *buf, size_t nb);
	ssize_t (*write)(void *d, const struct iiod_buf *buf, size_t nb);
	ssize_t (*discard)(void *d, size_t bytes);
};

/* Create / Destroy IIOD Responder. */
struct iiod_responder *
iiod_responder_create(const struct iiod_responder_ops *ops, void *d);
void iiod_responder_destroy(struct iiod_responder *responder);

/* Wait until the iiod_responder stops. */
void iiod_responder_wait_done(struct iiod_responder *responder);

/* Create a reader instance, to be used for I/O. */
struct iiod_reader *
iiod_responder_create_reader(struct iiod_responder *responder);

/* Create a reader suitable for responding to the given command. */
struct iiod_reader *
iiod_command_create_reader(const struct iiod_command *cmd,
			   struct iiod_command_data *data);

/* Remove queued asynchronous requests for commands or responses. */
void iiod_reader_cancel(struct iiod_reader *reader);

/* Destroy I/O instance. */
void iiod_reader_destroy(struct iiod_reader *reader);

/* Read the command's additional data, if any. */
ssize_t iiod_command_data_read(struct iiod_command_data *data,
			       const struct iiod_buf *buf);

/* Send command or response to the remote */
int iiod_reader_send_command(struct iiod_reader *reader,
			     const struct iiod_command *cmd,
			     const struct iiod_buf *buf, size_t nb);
int iiod_reader_send_response(struct iiod_reader *reader, intptr_t code,
			      const struct iiod_buf *buf, size_t nb);

/* Send command, then read the response. */
int iiod_reader_exec_command(struct iiod_reader *reader,
			     const struct iiod_command *cmd,
			     const struct iiod_buf *cmd_buf,
			     const struct iiod_buf *buf);

/* Simplified version of iiod_reader_exec_command.
 * Send a simple command then read the response code. */
static inline int
iiod_reader_exec_simple_command(struct iiod_reader *reader,
				const struct iiod_command *cmd)
{
	return iiod_reader_exec_command(reader, cmd, NULL, NULL);
}

typedef void (*iiod_async_cleanup_t)(void *, ssize_t ret);

/* Asynchronous variants of the functions above */
int iiod_reader_send_command_async(struct iiod_reader *reader,
				   const struct iiod_command *cmd,
				   const struct iiod_buf *buf, size_t nb,
				   iiod_async_cleanup_t cleanup_cb, void *d);
int iiod_reader_send_response_async(struct iiod_reader *reader, intptr_t code,
				    const struct iiod_buf *buf, size_t nb,
				    iiod_async_cleanup_t cleanup_cb, void *d);

/* Wait for an async. command or response to be done sending */
void iiod_reader_wait_for_command_done(struct iiod_reader *reader);

/* Simplified version of iiod_reader_send_response_async, to just send a code. */
static inline int
iiod_reader_send_response_code(struct iiod_reader *reader, intptr_t code)
{
	return iiod_reader_send_response(reader, code, NULL, 0);
}

/* Receive a response for a previously sent command */
intptr_t iiod_reader_get_response(struct iiod_reader *reader,
				  const struct iiod_buf *buf, size_t nb);

/* Simplififed version of iiod_reader_get_response */
static inline intptr_t
iiod_reader_get_response_code(struct iiod_reader *reader)
{
	return iiod_reader_get_response(reader, NULL, 0);
}

/* Asynchronous variant of iiod_reader_get_response */
void iiod_reader_get_response_async(struct iiod_reader *reader,
				    const struct iiod_buf *buf, size_t nb);

/* Wait for iiod_reader_get_response_async to be done. */
intptr_t iiod_reader_wait_for_response(struct iiod_reader *reader);

/* Atomically get the code of a previously queued response request
 * (with iiod_reader_get_response_async), and queue a new response request. */
intptr_t iiod_reader_get_and_request_response(struct iiod_reader *reader,
					      const struct iiod_buf *buf, size_t nb);

#endif /* __IIOD_RESPONDER_H__ */
