/*
 * Copyright (c) 2024 Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

 /**
  * @brief CYW20829 driver.
  */

#include <errno.h>
#include <stddef.h>
#include <string.h>

#include <zephyr/arch/cpu.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/drivers/bluetooth/hci_driver.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/init.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>

#include <wiced_bt_stack_platform.h>
#include <cybt_platform_config.h>
#include <cybt_platform_trace.h>
#include <cybt_platform_hci.h>
#include <cybt_platform_task.h>

#include <cyabs_rtos.h>
#include <cybt_result.h>

#define BT_DBG_ENABLED IS_ENABLED(CONFIG_BT_DEBUG_HCI_DRIVER)
#define LOG_LEVEL  CONFIG_BT_HCI_DRIVER_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(cyw208xx);


#define DT_DRV_COMPAT infineon_cyw208xx_hci


#define PACKET_TYPE_HCI_COMMAND     0X1
#define PACKET_TYPE_HCI_ACL_DATA    0x2
#define PACKET_TYPE_HCI_SYNCHRONOUS 0X3
#define PACKET_TYPE_HCI_EVENT       0X4


enum {
	BT_HCI_VND_OP_DOWNLOAD_MINIDRIVER = 0xFC2E,
	BT_HCI_VND_OP_WRITE_RAM = 0xFC4C,
	BT_HCI_VND_OP_LAUNCH_RAM = 0xFC4E,
	BT_HCI_VND_OP_UPDATE_BAUDRATE = 0xFC18,
};


/* Externs for CY43xxx controller FW */
extern const uint8_t brcm_patchram_buf[];
extern const int brcm_patch_ram_length;

#define CYBSP_BT_PLATFORM_CFG_SLEEP_MODE_LP_ENABLED   (1)

static K_SEM_DEFINE(hci_sem, 1, 1);

/******************************************************************************
 *                          Function Declarations
 ******************************************************************************/
extern void host_stack_platform_interface_init(void);
extern void cybt_platform_hci_wait_for_boot_fully_up(bool is_from_isr);
extern uint8_t *host_stack_get_acl_to_lower_buffer(wiced_bt_transport_t transport, uint32_t size);
extern wiced_result_t host_stack_send_acl_to_lower(wiced_bt_transport_t transport,
						   uint8_t *p_data, uint16_t len);
extern wiced_result_t host_stack_send_cmd_to_lower(uint8_t *p_cmd, uint16_t cmd_len);
extern wiced_result_t host_stack_send_iso_to_lower(uint8_t *p_data, uint16_t len);
extern cybt_result_t cybt_platform_msg_to_bt_task(const uint16_t msg, bool is_from_isr);
extern void cybt_bttask_deinit(void);

static int bt_firmware_download(const uint8_t *firmware_image, uint32_t size)
{
	uint8_t *data = (uint8_t *)firmware_image;
	volatile uint32_t remaining_length = size;
	struct net_buf *buf;
	int err;

	LOG_DBG("Executing Fw downloading for CYW208xx device");

	/* The firmware image (.hcd format) contains a collection of hci_write_ram
	 * command + a block of the image, followed by a hci_write_ram image at the end.
	 * Parse and send each individual command and wait for the response. This is to
	 * ensure the integrity of the firmware image sent to the bluetooth chip.
	 */
	while (remaining_length) {
		size_t data_length = data[2]; /* data length from firmware image block */
		uint16_t op_code = *(uint16_t *)data;

		/* Allocate buffer for hci_write_ram/hci_launch_ram command. */
		buf = bt_hci_cmd_create(op_code, data_length);
		if (buf == NULL) {
			LOG_ERR("Unable to allocate command buffer");
			return err;
		}

		/* Add data part of packet */
		net_buf_add_mem(buf, &data[3], data_length);

		/* Send hci_write_ram command. */
		err = bt_hci_cmd_send_sync(op_code, buf, NULL);
		if (err) {
			return err;
		}

		switch (op_code) {
		case BT_HCI_VND_OP_WRITE_RAM:
			/* Update remaining length and data pointer:
			 * content of data length + 2 bytes of opcode and 1 byte of data length.
			 */
			data += data_length + 3;
			remaining_length -= data_length + 3;
			break;

		case BT_HCI_VND_OP_LAUNCH_RAM:
			remaining_length = 0;
			break;

		default:
			return -ENOMEM;
		}
	}

	LOG_DBG("Fw downloading complete");
	return 0;
}


static int cyw208xx_open(void)
{
	int err;

	cybt_platform_task_init((void *)NULL);

	k_msleep(1); /* sleep to allow bt_task_handler starts */

	/* Send HCI_RESET */
	err = bt_hci_cmd_send_sync(BT_HCI_OP_RESET, NULL, NULL);
	if (err) {
		return err;
	}

	/* BT firmware download */
	err = bt_firmware_download(brcm_patchram_buf, (uint32_t)brcm_patch_ram_length);
	if (err) {
		return err;
	}

	/* Waiting when BLE up after firmware launch */
	cybt_platform_hci_wait_for_boot_fully_up(false);

	return 0;
}


static int cyw208xx_close(void)
{
	/* Send SHUTDOWN event, BT task will release resources and tervinate task */
	cybt_platform_msg_to_bt_task(BT_EVT_TASK_SHUTDOWN, false);

	cybt_bttask_deinit();
	return 0;
}

static int cyw208xx_send(struct net_buf *buf)
{
	int ret = 0;

	k_sem_take(&hci_sem, K_FOREVER);

	LOG_DBG("buf %p type %u len %u", buf, bt_buf_get_type(buf), buf->len);

	switch (bt_buf_get_type(buf)) {
	case BT_BUF_ACL_OUT:
		uint8_t *p_bt_msg = host_stack_get_acl_to_lower_buffer(BT_TRANSPORT_LE, buf->len);

		memcpy(p_bt_msg, buf->data, buf->len);
		ret = host_stack_send_acl_to_lower(BT_TRANSPORT_LE, p_bt_msg, buf->len);
		break;

	case BT_BUF_CMD:
		ret = host_stack_send_cmd_to_lower(buf->data, buf->len);
		break;

	case BT_BUF_ISO_OUT:
		ret = host_stack_send_iso_to_lower(buf->data, buf->len);
		break;

	default:
		LOG_ERR("Unknown type %u", bt_buf_get_type(buf));
		ret = EIO;
		goto done;
	}

	LOG_HEXDUMP_DBG(buf->data, buf->len, "Final HCI buffer:");

	if (ret) {
		LOG_ERR("SPI write error %d", ret);
	}

done:
	k_sem_give(&hci_sem);
	net_buf_unref(buf);
	return ret ? -EIO : 0;
}


static const struct bt_hci_driver drv = {
	.name = "CYW208xx",
	.bus = BT_HCI_DRIVER_BUS_VIRTUAL,
	.quirks = BT_QUIRK_NO_RESET,
	.open = cyw208xx_open,
	.close = cyw208xx_close,
	.send = cyw208xx_send,
};


static int cyw208xx_hci_init(void)
{
	const cybt_platform_config_t cybsp_bt_platform_cfg = {
		.hci_config = {
			.hci_transport = CYBT_HCI_IPC,
		},

		.controller_config = {
			.sleep_mode = {
				.sleep_mode_enabled = CYBSP_BT_PLATFORM_CFG_SLEEP_MODE_LP_ENABLED,
			},
		}
	};

	/* Configure platform specific settings for the BT device */
	cybt_platform_config_init(&cybsp_bt_platform_cfg);

	/* Register HCI driver to the Bluetooth stack. */
	bt_hci_driver_register(&drv);

	return 0;
}

SYS_INIT(cyw208xx_hci_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);


wiced_result_t wiced_bt_dev_vendor_specific_command(uint16_t opcode, uint8_t param_len,
	uint8_t *p_param_buf, wiced_bt_dev_vendor_specific_command_complete_cback_t *p_cback)
{
	struct net_buf *buf = NULL;

	/* Allocate a HCI command buffer */
	buf = bt_hci_cmd_create(opcode, param_len);
	if (!buf) {
		LOG_ERR("Unable to allocate buffer");
		return WICED_NO_MEMORY;
	}

	/* Add data part of packet */
	net_buf_add_mem(buf, p_param_buf, param_len);
	bt_hci_cmd_send(opcode, buf);

	return WICED_BT_SUCCESS;
}

void wiced_bt_process_hci(hci_packet_type_t pti, uint8_t *pData, uint32_t length)
{
	struct net_buf *buf = NULL;
	size_t buf_tailroom = 0;

	switch (pti) {
	case HCI_PACKET_TYPE_EVENT:
		buf = bt_buf_get_evt(pData[0], 0, K_NO_WAIT);
		if (!buf) {
			LOG_ERR("Failed to allocate the buffer for RX: EVENT ");
			return;
		}
		break;

	case HCI_PACKET_TYPE_ACL:
		buf = bt_buf_get_rx(BT_BUF_ACL_IN, K_NO_WAIT);
		if (!buf) {
			LOG_ERR("Failed to allocate the buffer for RX: ACL ");
			return;
		}
		bt_buf_set_type(buf, BT_BUF_ACL_IN);
		break;

	case HCI_PACKET_TYPE_SCO:
		/* NA */
		break;

	case HCI_PACKET_TYPE_ISO:
		buf = bt_buf_get_rx(BT_BUF_ISO_IN, K_NO_WAIT);
		if (!buf) {
			LOG_ERR("Failed to allocate the buffer for RX: ISO ");
			return;
		}
		break;

	default:
		return;

	}

	buf_tailroom = net_buf_tailroom(buf);
	if (buf_tailroom < length) {
		LOG_WRN("Not enough space for rx data");
		return;
	}

	net_buf_add_mem(buf, pData, length);
	bt_recv(buf);
}

void wiced_bt_process_hci_events(uint8_t *pData, uint32_t length)
{
	wiced_bt_process_hci(HCI_PACKET_TYPE_EVENT, pData, length);
}

void wiced_bt_process_acl_data(uint8_t *pData, uint32_t length)
{
	wiced_bt_process_hci(HCI_PACKET_TYPE_ACL, pData, length);
}

void wiced_bt_process_isoc_data(uint8_t *pData, uint32_t length)
{
	wiced_bt_process_hci(HCI_PACKET_TYPE_ISO, pData, length);
}


/* Keep below empty functions, used in btstack_integration assets for Wiced BT stack. */
void wiced_bt_stack_indicate_lower_tx_complete(void)
{
	/* NA for Zephyr */
}

void wiced_bt_stack_init_internal(wiced_bt_management_cback_t mgmt_cback,
				  wiced_bt_internal_post_stack_init_cb post_stack_cb,
				  wiced_bt_internal_stack_evt_handler_cb evt_handler_cb)
{
	/* NA for Zephyr */
}

void wiced_bt_stack_shutdown(void)
{
	/* NA for Zephyr */
}

void wiced_bt_process_timer(void)
{
	/* NA for Zephyr */
}
