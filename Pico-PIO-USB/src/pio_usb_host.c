/**
 * Copyright (c) 2021 sekigon-gonnoc
 *                    Ha Thach (thach@tinyusb.org)
 */

#pragma GCC push_options
#pragma GCC optimize("-O3")

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "hardware/sync.h"

#include "pio_usb.h"
#include "pio_usb_ll.h"
#include "usb_crc.h"
#include "usb_rx.pio.h"
#include "usb_tx.pio.h"

static alarm_pool_t *_alarm_pool = NULL;
static repeating_timer_t sof_rt;
// The sof_count may be incremented and then read on different cores.
static volatile uint32_t sof_count = 0;
static bool timer_active;

static volatile bool cancel_timer_flag;
static volatile bool start_timer_flag;
static __unused uint32_t int_stat;

static bool sof_timer(repeating_timer_t *_rt);

//--------------------------------------------------------------------+
// Application API
//--------------------------------------------------------------------+

static void start_timer(alarm_pool_t *alarm_pool) {
  if (timer_active) {
    return;
  }

  if (alarm_pool != NULL) {
    alarm_pool_add_repeating_timer_us(alarm_pool, -1000, sof_timer, NULL,
                                      &sof_rt);
  }

  timer_active = true;
}

usb_device_t *pio_usb_host_init(const pio_usb_configuration_t *c) {
  pio_port_t *pp = PIO_USB_PIO_PORT(0);
  root_port_t *root = PIO_USB_ROOT_PORT(0);

  pio_usb_bus_init(pp, c, root);
  root->mode = PIO_USB_MODE_HOST;

  float const cpu_freq = (float)clock_get_hz(clk_sys);
  pio_calculate_clkdiv_from_float(cpu_freq / 48000000,
                                  &pp->clk_div_fs_tx.div_int,
                                  &pp->clk_div_fs_tx.div_frac);
  pio_calculate_clkdiv_from_float(cpu_freq / 6000000,
                                  &pp->clk_div_ls_tx.div_int,
                                  &pp->clk_div_ls_tx.div_frac);

  pio_calculate_clkdiv_from_float(cpu_freq / 96000000,
                                  &pp->clk_div_fs_rx.div_int,
                                  &pp->clk_div_fs_rx.div_frac);
  pio_calculate_clkdiv_from_float(cpu_freq / 12000000,
                                  &pp->clk_div_ls_rx.div_int,
                                  &pp->clk_div_ls_rx.div_frac);

  if (!c->skip_alarm_pool) {
    _alarm_pool = c->alarm_pool;
    if (!_alarm_pool) {
      _alarm_pool = alarm_pool_create(2, 1);
    }
  }
  start_timer(_alarm_pool);

  return &pio_usb_device[0];
}

//--------------------------------------------------------------------+
// Bus functions
//--------------------------------------------------------------------+

static void __no_inline_not_in_flash_func(override_pio_program)(PIO pio, const pio_program_t* program, uint offset) {
    for (uint i = 0; i < program->length; ++i) {
      uint16_t instr = program->instructions[i];
      pio->instr_mem[offset + i] =
          pio_instr_bits_jmp != _pio_major_instr_bits(instr) ? instr
                                                             : instr + offset;
    }
}

static void __no_inline_not_in_flash_func(configure_lowspeed_host)(
    pio_port_t const *pp, root_port_t *port) {
  override_pio_program(pp->pio_usb_tx, pp->ls_tx_program, pp->offset_tx);
  SM_SET_CLKDIV(pp->pio_usb_tx, pp->sm_tx, pp->clk_div_ls_tx);

  pio_sm_set_jmp_pin(pp->pio_usb_rx, pp->sm_rx, port->pin_dm);
  SM_SET_CLKDIV_MAXSPEED(pp->pio_usb_rx, pp->sm_rx);

  pio_sm_set_jmp_pin(pp->pio_usb_rx, pp->sm_eop, port->pin_dp);
  pio_sm_set_in_pins(pp->pio_usb_rx, pp->sm_eop, port->pin_dm);
  SM_SET_CLKDIV(pp->pio_usb_rx, pp->sm_eop, pp->clk_div_ls_rx);

  usb_tx_configure_pins(pp->pio_usb_tx, pp->sm_tx, port->pin_dp, port->pin_dm);
}

static void __no_inline_not_in_flash_func(configure_root_port)(
    pio_port_t *pp, root_port_t *root) {
    configure_lowspeed_host(pp, root);
}

static void __no_inline_not_in_flash_func(restore_fs_bus)(const pio_port_t *pp) {
  // change bus speed to full-speed
  pio_sm_set_enabled(pp->pio_usb_tx, pp->sm_tx, false);
  SM_SET_CLKDIV(pp->pio_usb_tx, pp->sm_tx, pp->clk_div_fs_tx);

  pio_sm_set_enabled(pp->pio_usb_rx, pp->sm_rx, false);
  SM_SET_CLKDIV_MAXSPEED(pp->pio_usb_rx, pp->sm_rx);
  pio_sm_set_enabled(pp->pio_usb_rx, pp->sm_rx, true);

  pio_sm_set_enabled(pp->pio_usb_rx, pp->sm_eop, false);
  SM_SET_CLKDIV(pp->pio_usb_rx, pp->sm_eop, pp->clk_div_fs_rx);
  pio_sm_set_enabled(pp->pio_usb_rx, pp->sm_eop, true);
}

// Time about 1us ourselves so it lives in RAM.
static void __not_in_flash_func(busy_wait_1_us)(void) {
  uint32_t start = timer_hw->timerawl;
  while (timer_hw->timerawl == start) {
      tight_loop_contents();
  }
}

static bool __no_inline_not_in_flash_func(connection_check)(root_port_t *port) {
  if (pio_usb_bus_get_line_state(port) == PORT_PIN_SE0) {
    busy_wait_1_us();

    if (pio_usb_bus_get_line_state(port) == PORT_PIN_SE0) {
      busy_wait_1_us();
      // device disconnect
      port->connected = false;
      port->suspended = true;
      port->ints |= PIO_USB_INTS_DISCONNECT_BITS;

      // failed/retired all queuing transfer in this root
      uint8_t root_idx = port - PIO_USB_ROOT_PORT(0);
      for (int ep_idx = 0; ep_idx < PIO_USB_EP_POOL_CNT; ep_idx++) {
        endpoint_t *ep = PIO_USB_ENDPOINT(ep_idx);
        if ((ep->root_idx == root_idx) && ep->size && ep->has_transfer) {
          pio_usb_ll_transfer_complete(ep, PIO_USB_INTS_ENDPOINT_ERROR_BITS);
        }
      }

      return false;
    }
  }

  return true;
}

//--------------------------------------------------------------------+
// SOF
//--------------------------------------------------------------------+
static int usb_setup_transaction(pio_port_t *pp, endpoint_t *ep);
static int usb_in_transaction(pio_port_t *pp, endpoint_t *ep);
static int usb_out_transaction(pio_port_t *pp, endpoint_t *ep);

void __not_in_flash_func(pio_usb_host_frame)(void) {
  if (!timer_active) {
    return;
  }
  static uint8_t sof_packet[4] = {USB_SYNC, USB_PID_SOF, 0x00, 0x10};

  pio_port_t *pp = PIO_USB_PIO_PORT(0);

  // Send SOF
  for (int root_idx = 0; root_idx < PIO_USB_ROOT_PORT_CNT; root_idx++) {
    root_port_t *root = PIO_USB_ROOT_PORT(root_idx);
    if (!(root->initialized && root->connected && !root->suspended &&
          connection_check(root))) {
      continue;
    }
    configure_root_port(pp, root);
    pio_usb_bus_usb_transfer(pp, sof_packet, 4);
  }

  // Carry out all queued endpoint transaction
  for (int root_idx = 0; root_idx < PIO_USB_ROOT_PORT_CNT; root_idx++) {
    root_port_t *root = PIO_USB_ROOT_PORT(root_idx);
    if (!(root->initialized && root->connected && !root->suspended)) {
      continue;
    }

    configure_root_port(pp, root);

    for (int ep_pool_idx = 0; ep_pool_idx < PIO_USB_EP_POOL_CNT;
         ep_pool_idx++) {
      endpoint_t *ep = PIO_USB_ENDPOINT(ep_pool_idx);
      if ((ep->root_idx == root_idx) && ep->size) {
        bool const is_periodic = ((ep->attr & 0x03) == EP_ATTR_INTERRUPT);

        if (is_periodic && (ep->interval_counter > 0)) {
          ep->interval_counter--;
          continue;
        }

        if (ep->has_transfer && !ep->transfer_aborted) {
          ep->transfer_started = true;

          if (ep->need_pre) {
            pp->need_pre = true;
          }

          if (ep->ep_num == 0 && ep->data_id == USB_PID_SETUP) {
            usb_setup_transaction(pp, ep);
          } else {
            if (ep->ep_num & EP_IN) {
              usb_in_transaction(pp, ep);
            } else {
              usb_out_transaction(pp, ep);
            }

            if (is_periodic) {
              ep->interval_counter = ep->interval - 1;
            }
          }

          if (ep->need_pre) {
            pp->need_pre = false;
            restore_fs_bus(pp);
          }

          ep->transfer_started = false;
        }
      }
    }
  }

  // check for new connection to root hub
  for (int root_idx = 0; root_idx < PIO_USB_ROOT_PORT_CNT; root_idx++) {
    root_port_t *root = PIO_USB_ROOT_PORT(root_idx);
    if (root->initialized && !root->connected) {
      port_pin_status_t const line_state = pio_usb_bus_get_line_state(root);
      if (line_state == PORT_PIN_FS_IDLE || line_state == PORT_PIN_LS_IDLE) {
        root->is_fullspeed = (line_state == PORT_PIN_FS_IDLE);
        root->connected = true;
        root->suspended = true; // need a bus reset before operating
        root->ints |= PIO_USB_INTS_CONNECT_BITS;
      }
    }
  }

  // Invoke IRQHandler if interrupt status is set
  for (uint8_t root_idx = 0; root_idx < PIO_USB_ROOT_PORT_CNT; root_idx++) {
    if (PIO_USB_ROOT_PORT(root_idx)->ints) {
      pio_usb_host_irq_handler(root_idx);
    }
  }

  sof_count++;

  // SOF counter is 11-bit
  uint16_t const sof_count_11b = sof_count & 0x7ff;
  sof_packet[2] = sof_count_11b & 0xff;
  sof_packet[3] = (calc_usb_crc5(sof_count_11b) << 3) | (sof_count_11b >> 8);
}

static bool __no_inline_not_in_flash_func(sof_timer)(repeating_timer_t *_rt) {
  (void)_rt;

  pio_usb_host_frame();

  return true;
}

//--------------------------------------------------------------------+
// Host Controller functions
//--------------------------------------------------------------------+

void pio_usb_host_port_reset_start(uint8_t root_idx) {
  root_port_t *root = PIO_USB_ROOT_PORT(root_idx);
  pio_port_t *pp = PIO_USB_PIO_PORT(0);

  // bus is not operating while in reset
  root->suspended = true;

  // Force line state to SE0
  pio_sm_set_pins_with_mask(pp->pio_usb_tx, pp->sm_tx, 0,
                            (1 << root->pin_dp) | (1 << root->pin_dm));
  pio_sm_set_pindirs_with_mask(pp->pio_usb_tx, pp->sm_tx,
                               (1 << root->pin_dp) | (1 << root->pin_dm),
                               (1 << root->pin_dp) | (1 << root->pin_dm));
}

void pio_usb_host_port_reset_end(uint8_t root_idx) {
  root_port_t *root = PIO_USB_ROOT_PORT(root_idx);
  pio_port_t *pp = PIO_USB_PIO_PORT(0);

  // line state to input
  pio_sm_set_pindirs_with_mask(pp->pio_usb_tx, pp->sm_tx, 0,
                               (1 << root->pin_dp) | (1 << root->pin_dm));

  busy_wait_us(100); // TODO check if this is neccessary

  // bus back to operating
  root->suspended = false;
}

void pio_usb_host_close_device(uint8_t root_idx, uint8_t device_address) {
  for (int ep_pool_idx = 0; ep_pool_idx < PIO_USB_EP_POOL_CNT; ep_pool_idx++) {
    endpoint_t *ep = PIO_USB_ENDPOINT(ep_pool_idx);
    if ((ep->root_idx == root_idx) && (ep->dev_addr == device_address) &&
        ep->size) {
      ep->size = 0;
      ep->has_transfer = false;
    }
  }
}

static inline __force_inline endpoint_t * _find_ep(uint8_t root_idx, 
                                                   uint8_t device_address, uint8_t ep_address) {
  for (int ep_pool_idx = 0; ep_pool_idx < PIO_USB_EP_POOL_CNT; ep_pool_idx++) {
    endpoint_t *ep = PIO_USB_ENDPOINT(ep_pool_idx);
    // note 0x00 and 0x80 are matched as control endpoint of opposite direction
    if ((ep->root_idx == root_idx) && (ep->dev_addr == device_address) &&
        ep->size &&
        ((ep->ep_num == ep_address) ||
         (((ep_address & 0x7f) == 0) && ((ep->ep_num & 0x7f) == 0)))) {
      return ep;
    }
  }

  return NULL;
}

bool pio_usb_host_endpoint_open(uint8_t root_idx, uint8_t device_address,
                                uint8_t const *desc_endpoint, bool need_pre) {
  const endpoint_descriptor_t *d = (const endpoint_descriptor_t *)desc_endpoint;
  for (int ep_pool_idx = 0; ep_pool_idx < PIO_USB_EP_POOL_CNT; ep_pool_idx++) {
    endpoint_t *ep = PIO_USB_ENDPOINT(ep_pool_idx);
    // ep size is used as valid indicator
    if (ep->size == 0) {
      pio_usb_ll_configure_endpoint(ep, desc_endpoint);
      ep->root_idx = root_idx;
      ep->dev_addr = device_address;
      ep->need_pre = need_pre;
      ep->is_tx = (d->epaddr & 0x80) ? false : true; // host endpoint out is tx
      return true;
    }
  }

  return false;
}

bool pio_usb_host_send_setup(uint8_t root_idx, uint8_t device_address,
                             uint8_t const setup_packet[8]) {
  endpoint_t *ep = _find_ep(root_idx, device_address, 0);

  ep->ep_num = 0; // setup is is OUT
  ep->data_id = USB_PID_SETUP;
  ep->is_tx = true;

  return pio_usb_ll_transfer_start(ep, (uint8_t *)setup_packet, 8);
}

bool pio_usb_host_endpoint_transfer(uint8_t root_idx, uint8_t device_address,
                                    uint8_t ep_address, uint8_t *buffer,
                                    uint16_t buflen) {
  endpoint_t *ep = _find_ep(root_idx, device_address, ep_address);
  // Control endpoint, address may switch between 0x00 <-> 0x80
  // therefore we need to update ep_num and is_tx
  if ((ep_address & 0x7f) == 0) {
    ep->ep_num = ep_address;
    ep->is_tx = ep_address == 0;
    ep->data_id = 1; // data and status always start with DATA1
  }

  return pio_usb_ll_transfer_start(ep, buffer, buflen);
}

//--------------------------------------------------------------------+
// Transaction helper
//--------------------------------------------------------------------+

static int __no_inline_not_in_flash_func(usb_in_transaction)(pio_port_t *pp,
                                                             endpoint_t *ep) {
  int res = 0;
  uint8_t expect_pid = (ep->data_id == 1) ? USB_PID_DATA1 : USB_PID_DATA0;

  pio_usb_bus_prepare_receive(pp);
  pio_usb_bus_send_token(pp, USB_PID_IN, ep->dev_addr, ep->ep_num);
  pio_usb_bus_start_receive(pp);

  int receive_len = pio_usb_bus_receive_packet_and_handshake(pp, USB_PID_ACK);
  uint8_t const receive_pid = pp->usb_rx_buffer[1];

  if (receive_len >= 0) {
    if (receive_pid == expect_pid) {
      memcpy(ep->app_buf, &pp->usb_rx_buffer[2], receive_len);
      pio_usb_ll_transfer_continue(ep, receive_len);
    }
  }

  pio_sm_set_enabled(pp->pio_usb_rx, pp->sm_rx, false);
  pp->usb_rx_buffer[0] = 0;
  pp->usb_rx_buffer[1] = 0;

  return res;
}

static int __no_inline_not_in_flash_func(usb_out_transaction)(pio_port_t *pp,
                                                              endpoint_t *ep) {
  int res = 0;

  uint16_t const xact_len = pio_usb_ll_get_transaction_len(ep);

  pio_usb_bus_prepare_receive(pp);
  pio_usb_bus_send_token(pp, USB_PID_OUT, ep->dev_addr, ep->ep_num);
  // ensure previous tx complete
  while ((pp->pio_usb_tx->irq & IRQ_TX_COMP_MASK) == 0) {
    continue;
  }

  pio_usb_bus_usb_transfer(pp, ep->buffer, xact_len + 4);
  pio_usb_bus_start_receive(pp);

  // pio_usb_bus_wait_handshake(pp);

  uint8_t const receive_token = pp->usb_rx_buffer[1];

  if (receive_token == USB_PID_ACK) {
    pio_usb_ll_transfer_continue(ep, xact_len);
  }

  pio_sm_set_enabled(pp->pio_usb_rx, pp->sm_rx, false);
  pp->usb_rx_buffer[0] = 0;
  pp->usb_rx_buffer[1] = 0;

  return res;
}

static int __no_inline_not_in_flash_func(usb_setup_transaction)(
    pio_port_t *pp,  endpoint_t *ep) {

  int res = 0;

  // Setup token
  pio_usb_bus_prepare_receive(pp);

  pio_usb_bus_send_token(pp, USB_PID_SETUP, ep->dev_addr, 0);
  // ensure previous tx complete
  while ((pp->pio_usb_tx->irq & IRQ_TX_COMP_MASK) == 0) {
    continue;
  }

  // Data
  ep->data_id = 0; // set to DATA0
  pio_usb_bus_usb_transfer(pp, ep->buffer, 12);

  // Handshake
  pio_usb_bus_start_receive(pp);
  // pio_usb_bus_wait_handshake(pp);

  ep->actual_len = 8;

  if (pp->usb_rx_buffer[0] == USB_SYNC && pp->usb_rx_buffer[1] == USB_PID_ACK) {
    pio_usb_ll_transfer_complete(ep, PIO_USB_INTS_ENDPOINT_COMPLETE_BITS);
  }

  pp->usb_rx_buffer[1] = 0; // reset buffer

  return res;
}

// IRQ Handler
static void __no_inline_not_in_flash_func(__pio_usb_host_irq_handler)(uint8_t root_id) {
  root_port_t *root = PIO_USB_ROOT_PORT(root_id);
  uint32_t const ints = root->ints;

  // clear all
  root->ints &= ~ints;
}

// weak alias to __pio_usb_host_irq_handler
void pio_usb_host_irq_handler(uint8_t root_id) __attribute__ ((weak, alias("__pio_usb_host_irq_handler")));

#pragma GCC pop_options
