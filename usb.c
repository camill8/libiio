// SPDX-License-Identifier: LGPL-2.1-or-later
/*
 * libiio - Library for interfacing industrial I/O (IIO) devices
 *
 * Copyright (C) 2015 - 2020 Analog Devices, Inc.
 * Author: Paul Cercueil <paul.cercueil@analog.com>
 */

#include "iio-backend.h"
#include "iio-config.h"
#include "iio-debug.h"
#include "iio-lock.h"
#include "iiod-client.h"

#include <ctype.h>
#include <errno.h>
#include <libusb.h>
#include <stdbool.h>
#include <string.h>

/* Endpoint for non-streaming operations */
#define EP_OPS		1

#define IIO_INTERFACE_NAME	"IIO"

struct iio_usb_ep_couple {
	unsigned char addr_in, addr_out;
	uint16_t pipe_id;
	bool in_use;

	const struct iio_device *dev;
};

struct iiod_client_pdata {
	struct iio_usb_ep_couple *ep;
	struct iiod_client *iiod_client;

	struct iio_mutex *lock;
	bool cancelled;
	struct libusb_transfer *transfer;

	struct iio_context_pdata *ctx_pdata;
};

struct iio_context_pdata {
	libusb_context *ctx;
	libusb_device_handle *hdl;
	uint16_t intrfc;

	/* Lock for endpoint reservation */
	struct iio_mutex *ep_lock;

	struct iio_usb_ep_couple *io_endpoints;
	uint16_t nb_ep_couples;

	unsigned int timeout_ms;

	struct iiod_client_pdata io_ctx;
};

struct iio_device_pdata {
	bool opened;
	struct iiod_client_pdata io_ctx;

	struct iiod_client_io *client_io;
};

static const unsigned int libusb_to_errno_codes[] = {
	[- LIBUSB_ERROR_INVALID_PARAM]	= EINVAL,
	[- LIBUSB_ERROR_ACCESS]		= EACCES,
	[- LIBUSB_ERROR_NO_DEVICE]	= ENODEV,
	[- LIBUSB_ERROR_NOT_FOUND]	= ENXIO,
	[- LIBUSB_ERROR_BUSY]		= EBUSY,
	[- LIBUSB_ERROR_TIMEOUT]	= ETIMEDOUT,
	[- LIBUSB_ERROR_OVERFLOW]	= EIO,
	[- LIBUSB_ERROR_PIPE]		= EPIPE,
	[- LIBUSB_ERROR_INTERRUPTED]	= EINTR,
	[- LIBUSB_ERROR_NO_MEM]		= ENOMEM,
	[- LIBUSB_ERROR_NOT_SUPPORTED]	= ENOSYS,
};

static unsigned int libusb_to_errno(int error)
{
	switch ((enum libusb_error) error) {
	case LIBUSB_ERROR_INVALID_PARAM:
	case LIBUSB_ERROR_ACCESS:
	case LIBUSB_ERROR_NO_DEVICE:
	case LIBUSB_ERROR_NOT_FOUND:
	case LIBUSB_ERROR_BUSY:
	case LIBUSB_ERROR_TIMEOUT:
	case LIBUSB_ERROR_PIPE:
	case LIBUSB_ERROR_INTERRUPTED:
	case LIBUSB_ERROR_NO_MEM:
	case LIBUSB_ERROR_NOT_SUPPORTED:
		return libusb_to_errno_codes[- (int) error];
	case LIBUSB_ERROR_IO:
	case LIBUSB_ERROR_OTHER:
	case LIBUSB_ERROR_OVERFLOW:
	default:
		return EIO;
	}
}

static const struct iiod_client_ops usb_iiod_client_ops;

static struct iio_context *
usb_create_context_from_args(const struct iio_context_params *params,
			     const char *args);
static int usb_context_scan(const struct iio_context_params *params,
			    struct iio_scan *scan);

static int usb_io_context_init(struct iiod_client_pdata *io_ctx)
{
	io_ctx->lock = iio_mutex_create();
	if (!io_ctx->lock)
		return -ENOMEM;

	return 0;
}

static void usb_io_context_exit(struct iiod_client_pdata *io_ctx)
{
	if (io_ctx->lock) {
		iio_mutex_destroy(io_ctx->lock);
		io_ctx->lock = NULL;
	}
}

static unsigned int usb_calculate_remote_timeout(unsigned int timeout)
{
	/* XXX(pcercuei): We currently hardcode timeout / 2 for the backend used
	 * by the remote. Is there something better to do here? */
	return timeout / 2;
}

#define USB_PIPE_CTRL_TIMEOUT 1000 /* These should not take long */

#define IIO_USD_CMD_RESET_PIPES 0
#define IIO_USD_CMD_OPEN_PIPE 1
#define IIO_USD_CMD_CLOSE_PIPE 2

static int usb_reset_pipes(struct iio_context_pdata *pdata)
{
	int ret;
/*
	int libusb_control_transfer(libusb_device_handle *devh,
			uint8_t bmRequestType,
			uint8_t bRequest,
			uint16_t wValue,
			uint16_t wIndex,
			unsigned char *data,
			uint16_t wLength,
			unsigned int timeout)
*/
	ret = libusb_control_transfer(pdata->hdl,
			LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
			IIO_USD_CMD_RESET_PIPES,
			0,
			pdata->intrfc,
			NULL,
			0,
			USB_PIPE_CTRL_TIMEOUT);
	if (ret < 0)
		return -(int) libusb_to_errno(ret);
	return 0;
}

static int usb_open_pipe(struct iio_context_pdata *pdata, uint16_t pipe_id)
{
	int ret;
/*
	libusb_device_handle *devh,
	uint8_t bmRequestType,
	uint8_t bRequest,
	uint16_t wValue,
	uint16_t wIndex,
	unsigned char *data,
	uint16_t wLength,
	unsigned int timeout)
*/
	ret = libusb_control_transfer(
			pdata->hdl,
			LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
			IIO_USD_CMD_OPEN_PIPE,
			pipe_id,
			pdata->intrfc,
			NULL,
			0,
			USB_PIPE_CTRL_TIMEOUT);
	if (ret < 0)
		return -(int) libusb_to_errno(ret);
	return 0;
}

static int usb_close_pipe(struct iio_context_pdata *pdata, uint16_t pipe_id)
{
	int ret;

	ret = libusb_control_transfer(pdata->hdl, LIBUSB_REQUEST_TYPE_VENDOR |
		LIBUSB_RECIPIENT_INTERFACE, IIO_USD_CMD_CLOSE_PIPE,
		pipe_id, pdata->intrfc, NULL, 0, USB_PIPE_CTRL_TIMEOUT);
	if (ret < 0)
		return -(int) libusb_to_errno(ret);
	return 0;
}

static int usb_reserve_ep_unlocked(const struct iio_device *dev)
{
	const struct iio_context *ctx = iio_device_get_context(dev);
	struct iio_context_pdata *pdata = iio_context_get_pdata(ctx);
	struct iio_device_pdata *ppdata = iio_device_get_pdata(dev);
	unsigned int i;

	for (i = 0; i < pdata->nb_ep_couples; i++) {
		struct iio_usb_ep_couple *ep = &pdata->io_endpoints[i];

		if (!ep->in_use) {
			ep->in_use = true;
			ep->dev = dev;
			ppdata->io_ctx.ep = ep;

			return 0;
		}
	}

	return -EBUSY;
}

static void usb_free_ep_unlocked(const struct iio_device *dev)
{
	const struct iio_context *ctx = iio_device_get_context(dev);
	struct iio_context_pdata *pdata = iio_context_get_pdata(ctx);
	unsigned int i;

	for (i = 0; i < pdata->nb_ep_couples; i++) {
		struct iio_usb_ep_couple *ep = &pdata->io_endpoints[i];

		if (ep->dev == dev) {
			ep->in_use = false;
			ep->dev = NULL;
			return;
		}
	}
}

static int usb_open(const struct iio_device *dev,
		size_t samples_count, bool cyclic)
{
	const struct iio_context *ctx = iio_device_get_context(dev);
	struct iio_context_pdata *ctx_pdata = iio_context_get_pdata(ctx);
	struct iio_device_pdata *pdata = iio_device_get_pdata(dev);
	const struct iio_context_params *params = iio_context_get_params(ctx);
	struct iiod_client *client;
	unsigned int timeout;
	int ret = -EBUSY;

	iio_mutex_lock(ctx_pdata->ep_lock);

	pdata->io_ctx.cancelled = false;

	if (pdata->opened)
		goto err_unlock;

	ret = usb_reserve_ep_unlocked(dev);
	if (ret)
		goto err_unlock;

	ret = usb_open_pipe(ctx_pdata, pdata->io_ctx.ep->pipe_id);
	if (ret) {
		dev_perror(dev, -ret, "Failed to open pipe");
		goto err_free_ep;
	}

	client = iiod_client_new(params, &pdata->io_ctx, &usb_iiod_client_ops);
	if (!client)
		goto err_close_pipe;

	iiod_client_mutex_lock(client);

	pdata->client_io = iiod_client_open_unlocked(client, dev,
						     samples_count, cyclic);
	if (IS_ERR(pdata->client_io)) {
		ret = PTR_ERR(pdata->client_io);
		goto err_unlock_client;
	}

	timeout = usb_calculate_remote_timeout(ctx_pdata->timeout_ms);

	ret = iiod_client_set_timeout(ctx_pdata->io_ctx.iiod_client, timeout);
	if (ret)
		goto err_usb_close;

	iiod_client_mutex_unlock(client);

	pdata->io_ctx.iiod_client = client;
	pdata->opened = true;
	iio_mutex_unlock(ctx_pdata->ep_lock);

	return 0;

err_usb_close:
	iiod_client_close_unlocked(pdata->client_io);
err_unlock_client:
	iiod_client_mutex_unlock(client);
	iiod_client_destroy(client);
err_close_pipe:
	usb_close_pipe(ctx_pdata, pdata->io_ctx.ep->pipe_id);
err_free_ep:
	usb_free_ep_unlocked(dev);
err_unlock:
	iio_mutex_unlock(ctx_pdata->ep_lock);

	return ret;
}

static int usb_close(const struct iio_device *dev)
{
	const struct iio_context *ctx = iio_device_get_context(dev);
	struct iio_context_pdata *ctx_pdata = iio_context_get_pdata(ctx);
	struct iio_device_pdata *pdata = iio_device_get_pdata(dev);
	struct iiod_client *client = pdata->io_ctx.iiod_client;
	int ret = -EBADF;

	iio_mutex_lock(ctx_pdata->ep_lock);
	if (!pdata->opened)
		goto out_unlock;

	iiod_client_mutex_lock(client);
	ret = iiod_client_close_unlocked(pdata->client_io);
	pdata->opened = false;

	iiod_client_mutex_unlock(client);
	iiod_client_destroy(client);

	usb_close_pipe(ctx_pdata, pdata->io_ctx.ep->pipe_id);

	usb_free_ep_unlocked(dev);

out_unlock:
	iio_mutex_unlock(ctx_pdata->ep_lock);
	return ret;
}

static ssize_t usb_read(const struct iio_device *dev, void *dst, size_t len,
		uint32_t *mask, size_t words)
{
	struct iio_device_pdata *pdata = iio_device_get_pdata(dev);
	struct iiod_client *client = pdata->io_ctx.iiod_client;

	return iiod_client_read(client, dev, dst, len, mask, words);
}

static ssize_t usb_write(const struct iio_device *dev,
		const void *src, size_t len)
{
	struct iio_device_pdata *pdata = iio_device_get_pdata(dev);
	struct iiod_client *client = pdata->io_ctx.iiod_client;

	return iiod_client_write(client, dev, src, len);
}

static ssize_t usb_get_buffer(const struct iio_device *dev,
			      void **addr_ptr, size_t bytes_used,
			      uint32_t *mask, size_t words)
{
	struct iio_device_pdata *pdata = iio_device_get_pdata(dev);

	return iiod_client_get_buffer(pdata->client_io, addr_ptr, bytes_used,
				      mask, words);
}

static ssize_t usb_read_dev_attr(const struct iio_device *dev,
		const char *attr, char *dst, size_t len, enum iio_attr_type type)
{
	const struct iio_context *ctx = iio_device_get_context(dev);
	struct iio_context_pdata *pdata = iio_context_get_pdata(ctx);
	struct iiod_client *client = pdata->io_ctx.iiod_client;

	return iiod_client_read_attr(client, dev, NULL, attr, dst, len, type);
}

static ssize_t usb_write_dev_attr(const struct iio_device *dev,
		const char *attr, const char *src, size_t len, enum iio_attr_type type)
{
	const struct iio_context *ctx = iio_device_get_context(dev);
	struct iio_context_pdata *pdata = iio_context_get_pdata(ctx);
	struct iiod_client *client = pdata->io_ctx.iiod_client;

	return iiod_client_write_attr(client, dev, NULL, attr, src, len, type);
}

static ssize_t usb_read_chn_attr(const struct iio_channel *chn,
		const char *attr, char *dst, size_t len)
{
	const struct iio_device *dev = iio_channel_get_device(chn);
	const struct iio_context *ctx = iio_device_get_context(dev);
	struct iio_context_pdata *pdata = iio_context_get_pdata(ctx);
	struct iiod_client *client = pdata->io_ctx.iiod_client;

	return iiod_client_read_attr(client, dev, chn, attr, dst, len, false);
}

static ssize_t usb_write_chn_attr(const struct iio_channel *chn,
		const char *attr, const char *src, size_t len)
{
	const struct iio_device *dev = iio_channel_get_device(chn);
	const struct iio_context *ctx = iio_device_get_context(dev);
	struct iio_context_pdata *pdata = iio_context_get_pdata(ctx);
	struct iiod_client *client = pdata->io_ctx.iiod_client;

	return iiod_client_write_attr(client, dev, chn, attr, src, len, false);
}

static int usb_set_kernel_buffers_count(const struct iio_device *dev,
		unsigned int nb_blocks)
{
	const struct iio_context *ctx = iio_device_get_context(dev);
	struct iio_context_pdata *pdata = iio_context_get_pdata(ctx);
	struct iiod_client *client = pdata->io_ctx.iiod_client;

	return iiod_client_set_kernel_buffers_count(client, dev, nb_blocks);
}

static int usb_set_timeout(struct iio_context *ctx, unsigned int timeout)
{
	struct iio_context_pdata *pdata = iio_context_get_pdata(ctx);
	unsigned int remote_timeout = usb_calculate_remote_timeout(timeout);
	struct iiod_client *client = pdata->io_ctx.iiod_client;

	int ret;

	ret = iiod_client_set_timeout(client, remote_timeout);
	if (!ret)
		pdata->timeout_ms = timeout;

	return ret;
}

static int usb_get_trigger(const struct iio_device *dev,
                const struct iio_device **trigger)
{
	const struct iio_context *ctx = iio_device_get_context(dev);
	struct iio_context_pdata *pdata = iio_context_get_pdata(ctx);
	struct iiod_client *client = pdata->io_ctx.iiod_client;

	return iiod_client_get_trigger(client, dev, trigger);
}

static int usb_set_trigger(const struct iio_device *dev,
                const struct iio_device *trigger)
{
	const struct iio_context *ctx = iio_device_get_context(dev);
	struct iio_context_pdata *pdata = iio_context_get_pdata(ctx);
	struct iiod_client *client = pdata->io_ctx.iiod_client;

	return iiod_client_set_trigger(client, dev, trigger);
}


static void usb_shutdown(struct iio_context *ctx)
{
	struct iio_context_pdata *pdata = iio_context_get_pdata(ctx);
	unsigned int nb_devices = iio_context_get_devices_count(ctx);
	unsigned int i;

	usb_io_context_exit(&pdata->io_ctx);

	for (i = 0; i < nb_devices; i++)
		usb_close(iio_context_get_device(ctx, i));

	iio_mutex_destroy(pdata->ep_lock);

	iiod_client_destroy(pdata->io_ctx.iiod_client);

	free(pdata->io_endpoints);

	for (i = 0; i < nb_devices; i++) {
		const struct iio_device *dev = iio_context_get_device(ctx, i);
		struct iio_device_pdata *ppdata = iio_device_get_pdata(dev);

		usb_io_context_exit(&ppdata->io_ctx);
		free(ppdata);
	}

	usb_reset_pipes(pdata); /* Close everything */

	libusb_close(pdata->hdl);
	libusb_exit(pdata->ctx);
}

static int iio_usb_match_interface(const struct libusb_config_descriptor *desc,
		struct libusb_device_handle *hdl, unsigned int intrfc)
{
	const struct libusb_interface *iface;
	unsigned int i;

	if (intrfc >= desc->bNumInterfaces)
		return -EINVAL;

	iface = &desc->interface[intrfc];

	for (i = 0; i < (unsigned int) iface->num_altsetting; i++) {
		const struct libusb_interface_descriptor *idesc =
			&iface->altsetting[i];
		char name[64];
		int ret;

		if (idesc->iInterface == 0)
			continue;

		ret = libusb_get_string_descriptor_ascii(hdl, idesc->iInterface,
				(unsigned char *) name, sizeof(name));
		if (ret < 0)
			return -(int) libusb_to_errno(ret);

		if (!strcmp(name, IIO_INTERFACE_NAME))
			return (int) i;
	}

	return -EPERM;
}

static int iio_usb_match_device(struct libusb_device *dev,
		struct libusb_device_handle *hdl,
		unsigned int *intrfc)
{
	struct libusb_config_descriptor *desc;
	unsigned int i;
	int ret;

	ret = libusb_get_active_config_descriptor(dev, &desc);
	if (ret)
		return -(int) libusb_to_errno(ret);

	for (i = 0, ret = -EPERM; ret == -EPERM &&
			i < desc->bNumInterfaces; i++)
		ret = iio_usb_match_interface(desc, hdl, i);

	libusb_free_config_descriptor(desc);
	if (ret < 0)
		return ret;

	prm_dbg(NULL, "Found IIO interface on device %u:%u using interface %u\n",
			libusb_get_bus_number(dev),
			libusb_get_device_address(dev), i - 1);

	*intrfc = i - 1;
	return ret;
}

static void usb_cancel(const struct iio_device *dev)
{
	struct iio_device_pdata *ppdata = iio_device_get_pdata(dev);

	iio_mutex_lock(ppdata->io_ctx.lock);
	if (ppdata->io_ctx.transfer && !ppdata->io_ctx.cancelled)
		libusb_cancel_transfer(ppdata->io_ctx.transfer);
	ppdata->io_ctx.cancelled = true;
	iio_mutex_unlock(ppdata->io_ctx.lock);
}

static const struct iio_backend_ops usb_ops = {
	.scan = usb_context_scan,
	.create = usb_create_context_from_args,
	.open = usb_open,
	.close = usb_close,
	.read = usb_read,
	.write = usb_write,
	.get_buffer = usb_get_buffer,
	.read_device_attr = usb_read_dev_attr,
	.read_channel_attr = usb_read_chn_attr,
	.write_device_attr = usb_write_dev_attr,
	.write_channel_attr = usb_write_chn_attr,
	.get_trigger = usb_get_trigger,
	.set_trigger = usb_set_trigger,
	.set_kernel_buffers_count = usb_set_kernel_buffers_count,
	.set_timeout = usb_set_timeout,
	.shutdown = usb_shutdown,

	.cancel = usb_cancel,
};

__api_export_if(WITH_USB_BACKEND_DYNAMIC)
const struct iio_backend iio_usb_backend = {
	.api_version = IIO_BACKEND_API_V1,
	.name = "usb",
	.uri_prefix = "usb:",
	.ops = &usb_ops,
	.default_timeout_ms = 5000,
};

static void LIBUSB_CALL sync_transfer_cb(struct libusb_transfer *transfer)
{
	int *completed = transfer->user_data;
	*completed = 1;
}

static int usb_sync_transfer(struct iio_context_pdata *pdata,
	struct iiod_client_pdata *io_ctx, unsigned int ep_type,
	char *data, size_t len, int *transferred)
{
	unsigned char ep;
	struct libusb_transfer *transfer = NULL;
	int completed = 0;
	int ret;

	/*
	 * If the size of the data to transfer is too big, the
	 * IOCTL_USBFS_SUBMITURB ioctl (called by libusb) might fail with
	 * errno set to ENOMEM, as the kernel might use contiguous allocation
	 * for the URB if the driver doesn't support scatter-gather.
	 * To prevent that, we support URBs of 1 MiB maximum. The iiod-client
	 * code will handle this properly and ask for a new transfer.
	 */
	if (len > 1 * 1024 * 1024)
		len = 1 * 1024 * 1024;

	if (ep_type == LIBUSB_ENDPOINT_IN)
		ep = io_ctx->ep->addr_in;
	else
		ep = io_ctx->ep->addr_out;

	/*
	 * For cancellation support the check whether the buffer has already been
	 * cancelled and the allocation as well as the assignment of the new
	 * transfer needs to happen in one atomic step. Otherwise it is possible
	 * that the cancellation is missed and transfer is not aborted.
	 */
	iio_mutex_lock(io_ctx->lock);
	if (io_ctx->cancelled) {
		ret = -EBADF;
		goto unlock;
	}

	transfer = libusb_alloc_transfer(0);
	if (!transfer) {
		ret = -ENOMEM;
		goto unlock;
	}

	transfer->user_data = &completed;

	libusb_fill_bulk_transfer(transfer, pdata->hdl, ep,
			(unsigned char *) data, (int) len, sync_transfer_cb,
			&completed, pdata->timeout_ms);
	transfer->type = LIBUSB_TRANSFER_TYPE_BULK;

	ret = libusb_submit_transfer(transfer);
	if (ret) {
		ret = -(int) libusb_to_errno(ret);
		libusb_free_transfer(transfer);
		goto unlock;
	}

	io_ctx->transfer = transfer;
unlock:
	iio_mutex_unlock(io_ctx->lock);
	if (ret)
		return ret;

	while (!completed) {
		ret = libusb_handle_events_completed(pdata->ctx, &completed);
		if (ret < 0) {
			if (ret == LIBUSB_ERROR_INTERRUPTED)
				continue;
			libusb_cancel_transfer(transfer);
			continue;
		}
	}

	switch (transfer->status) {
	case LIBUSB_TRANSFER_COMPLETED:
		*transferred = transfer->actual_length;
		ret = 0;
		break;
	case LIBUSB_TRANSFER_TIMED_OUT:
		ret = -ETIMEDOUT;
		break;
	case LIBUSB_TRANSFER_STALL:
		ret = -EPIPE;
		break;
	case LIBUSB_TRANSFER_NO_DEVICE:
		ret = -ENODEV;
		break;
	case LIBUSB_TRANSFER_CANCELLED:
		ret = -EBADF;
		break;
	default:
		ret = -EIO;
		break;
	}

	/* Same as above. This needs to be atomic in regards to usb_cancel(). */
	iio_mutex_lock(io_ctx->lock);
	io_ctx->transfer = NULL;
	iio_mutex_unlock(io_ctx->lock);

	libusb_free_transfer(transfer);

	return ret;
}

static ssize_t write_data_sync(struct iiod_client_pdata *ep,
			       const char *data, size_t len)
{
	int transferred, ret;

	ret = usb_sync_transfer(ep->ctx_pdata, ep, LIBUSB_ENDPOINT_OUT,
				(char *) data, len, &transferred);
	if (ret)
		return ret;
	else
		return (ssize_t) transferred;
}

static ssize_t read_data_sync(struct iiod_client_pdata *ep,
			      char *buf, size_t len)
{
	int transferred, ret;

	ret = usb_sync_transfer(ep->ctx_pdata, ep, LIBUSB_ENDPOINT_IN,
				buf, len, &transferred);
	if (ret)
		return ret;
	else
		return transferred;
}

static const struct iiod_client_ops usb_iiod_client_ops = {
	.write = write_data_sync,
	.read = read_data_sync,
	.read_line = read_data_sync,
};

static int usb_verify_eps(const struct libusb_interface_descriptor *iface)
{
	unsigned int i, eps = iface->bNumEndpoints;

	/* Check that we have an even number of endpoints, and that input/output
	 * endpoints are interleaved */

	if (eps < 2 || eps % 2)
		return -EINVAL;

	for (i = 0; i < eps; i += 2) {
		if (!(iface->endpoint[i + 0].bEndpointAddress
					& LIBUSB_ENDPOINT_IN))
			return -EINVAL;

		if (iface->endpoint[i + 1].bEndpointAddress
				& LIBUSB_ENDPOINT_IN)
			return -EINVAL;
	}

	return 0;
}

static int usb_get_string(libusb_device_handle *hdl, uint8_t idx,
			  char *buffer, size_t length)
{
	int ret;

	ret = libusb_get_string_descriptor_ascii(hdl, idx,
						 (unsigned char *) buffer,
						 (int) length);
	if (ret < 0) {
		buffer[0] = '\0';
		return -(int) libusb_to_errno(ret);
	}

	return 0;
}

static int usb_get_description(struct libusb_device_handle *hdl,
			       const struct libusb_device_descriptor *desc,
			       char *buffer, size_t length)
{
	char manufacturer[64], product[64], serial[64];
	ssize_t ret;

	manufacturer[0] = '\0';
	if (desc->iManufacturer > 0) {
		usb_get_string(hdl, desc->iManufacturer,
			       manufacturer, sizeof(manufacturer));
	}

	product[0] = '\0';
	if (desc->iProduct > 0) {
		usb_get_string(hdl, desc->iProduct,
			       product, sizeof(product));
	}

	serial[0] = '\0';
	if (desc->iSerialNumber > 0) {
		usb_get_string(hdl, desc->iSerialNumber,
			       serial, sizeof(serial));
	}

	ret = iio_snprintf(buffer, length,
			   "%04x:%04x (%s %s), serial=%s", desc->idVendor,
			   desc->idProduct, manufacturer, product, serial);
	if (ret < 0)
		return (int) ret;

	return 0;
}

static struct iio_context *
usb_create_context_with_attrs(libusb_device *usb_dev,
			      struct iio_context_pdata *pdata)
{
	struct libusb_version const *libusb_version = libusb_get_version();
	struct libusb_device_descriptor dev_desc;
	char vendor[64], product[64], serial[64],
	     uri[sizeof("usb:127.255.255")],
	     idVendor[5], idProduct[5], version[4],
	     lib_version[16], description[256];
	const char *attr_names[] = {
		"uri",
		"usb,vendor",
		"usb,product",
		"usb,serial",
		"usb,idVendor",
		"usb,idProduct",
		"usb,release",
		"usb,libusb",
	};
	char *attr_values[ARRAY_SIZE(attr_names)] = {
		uri,
		vendor,
		product,
		serial,
		idVendor,
		idProduct,
		version,
		lib_version,
	};
	struct iiod_client *client = pdata->io_ctx.iiod_client;

	libusb_get_device_descriptor(usb_dev, &dev_desc);

	usb_get_description(pdata->hdl, &dev_desc,
			    description, sizeof(description));

	iio_snprintf(uri, sizeof(uri), "usb:%d.%d.%u",
		     libusb_get_bus_number(usb_dev),
		     libusb_get_device_address(usb_dev),
		     (uint8_t)pdata->intrfc);
	usb_get_string(pdata->hdl, dev_desc.iManufacturer,
		       vendor, sizeof(vendor));
	usb_get_string(pdata->hdl, dev_desc.iProduct,
		       product, sizeof(product));
	usb_get_string(pdata->hdl, dev_desc.iSerialNumber,
		       serial, sizeof(serial));
	iio_snprintf(idVendor, sizeof(idVendor), "%04hx", dev_desc.idVendor);
	iio_snprintf(idProduct, sizeof(idProduct), "%04hx", dev_desc.idProduct);
	iio_snprintf(version, sizeof(version), "%1hhx.%1hhx",
		     (unsigned char)((dev_desc.bcdUSB >> 8) & 0xf),
		     (unsigned char)((dev_desc.bcdUSB >> 4) & 0xf));
	iio_snprintf(lib_version, sizeof(lib_version), "%i.%i.%i.%i%s",
		     libusb_version->major, libusb_version->minor,
		     libusb_version->micro, libusb_version->nano,
		     libusb_version->rc);

	return iiod_client_create_context(client,
					  &iio_usb_backend, description,
					  attr_names,
					  (const char **) attr_values,
					  ARRAY_SIZE(attr_names));
}

static struct iio_context * usb_create_context(const struct iio_context_params *params,
					       unsigned int bus,
					       uint16_t address, uint16_t intrfc)
{
	libusb_context *usb_ctx;
	libusb_device_handle *hdl = NULL;
	const struct libusb_interface_descriptor *iface;
	libusb_device *usb_dev;
	struct libusb_config_descriptor *conf_desc;
	libusb_device **device_list;
	struct iio_context *ctx;
	struct iio_context_pdata *pdata;
	uint16_t i;
	int ret;

	pdata = zalloc(sizeof(*pdata));
	if (!pdata) {
		prm_err(params, "Unable to allocate pdata\n");
		ret = -ENOMEM;
		goto err_set_errno;
	}

	pdata->ep_lock = iio_mutex_create();
	if (!pdata->ep_lock) {
		prm_err(params, "Unable to create mutex\n");
		ret = -ENOMEM;
		goto err_free_pdata;
	}

	ret = libusb_init(&usb_ctx);
	if (ret) {
		ret = -(int) libusb_to_errno(ret);
		prm_perror(params, -ret, "Unable to init libusb");
		goto err_destroy_ep_mutex;
	}

	ret = (int) libusb_get_device_list(usb_ctx, &device_list);
	if (ret < 0) {
		ret = -(int) libusb_to_errno(ret);
		prm_perror(params, -ret, "Unable to get usb device list");
		goto err_libusb_exit;
	}

	usb_dev = NULL;

	for (i = 0; device_list[i]; i++) {
		libusb_device *dev = device_list[i];

		if (bus == libusb_get_bus_number(dev) &&
			address == libusb_get_device_address(dev)) {
			usb_dev = dev;

			ret = libusb_open(usb_dev, &hdl);
			/*
			 * Workaround for libusb on Windows >= 8.1. A device
			 * might appear twice in the list with one device being
			 * bogus and only partially initialized. libusb_open()
			 * returns LIBUSB_ERROR_NOT_SUPPORTED for such devices,
			 * which should never happen for normal devices. So if
			 * we find such a device skip it and keep looking.
			 */
			if (ret == LIBUSB_ERROR_NOT_SUPPORTED) {
				prm_warn(params, "Skipping broken USB device. Please upgrade libusb.\n");
				usb_dev = NULL;
				continue;
			}

			break;
		}
	}

	libusb_free_device_list(device_list, true);

	if (!usb_dev || !hdl) {
		ret = -ENODEV;
		goto err_libusb_exit;
	}

	if (ret) {
		ret = -(int) libusb_to_errno(ret);
		prm_perror(params, -ret, "Unable to open device\n");
		goto err_libusb_exit;
	}

#if defined(LIBUSB_API_VERSION) && (LIBUSB_API_VERSION >= 0x01000016)
	libusb_set_auto_detach_kernel_driver(hdl, true);
#endif

	ret = libusb_claim_interface(hdl, intrfc);
	if (ret) {
		ret = -(int) libusb_to_errno(ret);
		prm_perror(params, -ret, "Unable to claim interface %u:%u:%u",
			   bus, address, intrfc);
		goto err_libusb_close;
	}

	ret = libusb_get_active_config_descriptor(usb_dev, &conf_desc);
	if (ret) {
		ret = -(int) libusb_to_errno(ret);
		prm_perror(params, -ret, "Unable to get config descriptor");
		goto err_libusb_close;
	}

	iface = &conf_desc->interface[intrfc].altsetting[0];

	ret = usb_verify_eps(iface);
	if (ret) {
		prm_perror(params, -ret, "Invalid configuration of endpoints");
		goto err_free_config_descriptor;
	}

	pdata->nb_ep_couples = iface->bNumEndpoints / 2;

	prm_dbg(params, "Found %hhu usable i/o endpoint couples\n",
		pdata->nb_ep_couples);

	pdata->io_endpoints = calloc(pdata->nb_ep_couples,
			sizeof(*pdata->io_endpoints));
	if (!pdata->io_endpoints) {
		prm_err(params, "Unable to allocate endpoints\n");
		ret = -ENOMEM;
		goto err_free_config_descriptor;
	}

	for (i = 0; i < pdata->nb_ep_couples; i++) {
		struct iio_usb_ep_couple *ep = &pdata->io_endpoints[i];

		ep->addr_in = iface->endpoint[i * 2 + 0].bEndpointAddress;
		ep->addr_out = iface->endpoint[i * 2 + 1].bEndpointAddress;
		ep->pipe_id = i;

		prm_dbg(params, "Couple %i with endpoints 0x%x / 0x%x\n", i,
			ep->addr_in, ep->addr_out);
	}

	pdata->ctx = usb_ctx;
	pdata->hdl = hdl;
	pdata->intrfc = intrfc;
	pdata->timeout_ms = params->timeout_ms;

	ret = usb_io_context_init(&pdata->io_ctx);
	if (ret)
		goto err_free_endpoints;

	/* We reserve the first I/O endpoint couple for global operations */
	pdata->io_ctx.ep = &pdata->io_endpoints[0];
	pdata->io_ctx.ep->in_use = true;

	pdata->io_ctx.ctx_pdata = pdata;

	pdata->io_ctx.iiod_client = iiod_client_new(params, &pdata->io_ctx,
						    &usb_iiod_client_ops);
	if (!pdata->io_ctx.iiod_client) {
		prm_err(params, "Unable to allocate memory\n");
		ret = -ENOMEM;
		goto err_io_context_exit;
	}

	ret = usb_reset_pipes(pdata);
	if (ret) {
		prm_perror(params, -ret, "Failed to reset pipes");
		goto err_free_iiod_client;
	}

	ret = usb_open_pipe(pdata, 0);
	if (ret) {
		prm_perror(params, -ret, "Failed to open control pipe");
		goto err_free_iiod_client;
	}

	ctx = usb_create_context_with_attrs(usb_dev, pdata);
	if (!ctx) {
		ret = -errno;
		goto err_reset_pipes;
	}

	libusb_free_config_descriptor(conf_desc);

	iio_context_set_pdata(ctx, pdata);

	for (i = 0; i < iio_context_get_devices_count(ctx); i++) {
		struct iio_device *dev = iio_context_get_device(ctx, i);
		struct iio_device_pdata *ppdata;

		ppdata = zalloc(sizeof(*ppdata));
		if (!ppdata) {
			prm_err(params, "Unable to allocate memory\n");
			ret = -ENOMEM;
			goto err_context_destroy;
		}

		iio_device_set_pdata(dev, ppdata);

		ret = usb_io_context_init(&ppdata->io_ctx);
		if (ret)
			goto err_context_destroy;

		ppdata->io_ctx.ctx_pdata = pdata;
	}

	return ctx;

err_context_destroy:
	iio_context_destroy(ctx);
	errno = -ret;
	return NULL;

err_reset_pipes:
	usb_reset_pipes(pdata); /* Close everything */
err_free_iiod_client:
	iiod_client_destroy(pdata->io_ctx.iiod_client);
err_io_context_exit:
	usb_io_context_exit(&pdata->io_ctx);
err_free_endpoints:
	free(pdata->io_endpoints);
err_free_config_descriptor:
	libusb_free_config_descriptor(conf_desc);
err_libusb_close:
	libusb_close(hdl);
err_libusb_exit:
	libusb_exit(usb_ctx);
err_destroy_ep_mutex:
	iio_mutex_destroy(pdata->ep_lock);
err_free_pdata:
	free(pdata);
err_set_errno:
	errno = -ret;
	return NULL;
}

static struct iio_context *
usb_create_context_from_args(const struct iio_context_params *params,
			     const char *args)
{
	long bus, address, intrfc;
	char *end;
	const char *ptr = args;
	/* keep MSVS happy by setting these to NULL */
	struct iio_scan *scan_ctx = NULL;
	bool scan;

	/* if uri is just "usb:" that means search for the first one */
	scan = !*ptr;
	if (scan) {
		scan_ctx = iio_scan(params, "usb");
		if (!scan_ctx)
			goto err_bad_uri;

		if (iio_scan_get_results_count(scan_ctx) != 1) {
			errno = ENXIO;
			goto err_bad_uri;
		}

		ptr = iio_scan_get_uri(scan_ctx, 0);
		ptr += sizeof("usb:") - 1;
	}

	if (!isdigit(*ptr))
		goto err_bad_uri;

	errno = 0;
	bus = strtol(ptr, &end, 10);
	if (ptr == end || *end != '.' || errno == ERANGE || bus < 0 || bus > UINT8_MAX)
		goto err_bad_uri;

	ptr = (const char *) ((uintptr_t) end + 1);
	if (!isdigit(*ptr))
		goto err_bad_uri;

	errno = 0;
	address = strtol(ptr, &end, 10);
	if (ptr == end || errno == ERANGE || address < 0 || address > UINT8_MAX)
		goto err_bad_uri;

	if (*end == '\0') {
		intrfc = 0;
	} else if (*end == '.') {
		ptr = (const char *) ((uintptr_t) end + 1);
		if (!isdigit(*ptr))
			goto err_bad_uri;

		errno = 0;
		intrfc = strtol(ptr, &end, 10);
		if (ptr == end || *end != '\0' || errno == ERANGE || intrfc < 0 || intrfc > UINT8_MAX)
			goto err_bad_uri;
	} else {
		goto err_bad_uri;
	}

	if (scan)
		iio_scan_destroy(scan_ctx);

	return usb_create_context(params, (unsigned int) bus,
			(uint16_t) address, (uint16_t) intrfc);

err_bad_uri:
	if (scan)
		iio_scan_destroy(scan_ctx);
	else
		errno = EINVAL;

	prm_err(params, "Bad URI: \'usb:%s\'\n", args);
	return NULL;
}

static int usb_add_context_info(struct iio_scan *scan,
				struct libusb_device *dev,
				struct libusb_device_handle *hdl,
				unsigned int intrfc)
{
	struct libusb_device_descriptor desc;
	char uri[sizeof("usb:127.255.255")];
	char description[256];
	int ret;

	libusb_get_device_descriptor(dev, &desc);

	ret = usb_get_description(hdl, &desc, description, sizeof(description));
	if (ret)
		return ret;

	iio_snprintf(uri, sizeof(uri), "usb:%d.%d.%u",
		libusb_get_bus_number(dev), libusb_get_device_address(dev),
		intrfc);

	return iio_scan_add_result(scan, description, uri);
}

static int usb_context_scan(const struct iio_context_params *params,
			    struct iio_scan *scan)
{
	libusb_device **device_list;
	libusb_context *ctx;
	unsigned int i;
	int ret;

	ret = libusb_init(&ctx);
	if (ret < 0)
		return -(int) libusb_to_errno(ret);

	ret = (int) libusb_get_device_list(ctx, &device_list);
	if (ret < 0) {
		ret = -(int) libusb_to_errno(ret);
		goto cleanup_libusb_exit;
	}

	for (i = 0; device_list[i]; i++) {
		struct libusb_device_handle *hdl;
		struct libusb_device *dev = device_list[i];
		unsigned int intrfc = 0;

		ret = libusb_open(dev, &hdl);
		if (ret)
			continue;

		if (!iio_usb_match_device(dev, hdl, &intrfc))
			ret = usb_add_context_info(scan, dev, hdl, intrfc);

		libusb_close(hdl);
		if (ret < 0)
			goto cleanup_free_device_list;
	}

	ret = 0;

cleanup_free_device_list:
	libusb_free_device_list(device_list, true);
cleanup_libusb_exit:
	libusb_exit(ctx);
	return ret;
}
