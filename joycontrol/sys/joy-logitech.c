/*
 *  joy-logitech.c  Version 1.2
 *
 *  Copyright (c) 1998-1999 Vojtech Pavlik
 */

/*
 * This is a module for the Linux joystick driver, supporting
 * Logitech ADI joystick family.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or 
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 * 
 * Should you need to contact me, the author, you can do so either by
 * e-mail - mail your message to <vojtech@ucw.cz>, or by paper mail:
 * Vojtech Pavlik, Ucitelska 1576, Prague 8, 182 00 Czech Republic
 */

#include <asm/io.h>
#include <asm/system.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <joystick.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/malloc.h>

/*
 * Times array sizes, flags, ids.
 */

#define JS_LT_MAX_START		400
#define JS_LT_MAX_STROBE	45

#define JS_LT_MAX_LENGTH	256
#define JS_LT_MIN_LENGTH	8
#define JS_LT_MIN_ID_LENGTH	66
#define JS_LT_MAX_NAME_LENGTH	16

#define JS_LT_SYNC_DELAY	10000
#define JS_LT_STATUS_DELAY	10000
#define JS_LT_EXTRA_DELAY	10000

#define JS_LT_FLAG_CHAIN	0x01
#define JS_LT_FLAG_HAT		0x04
#define JS_LT_FLAG_10BIT	0x08

#define JS_LT_ID_TPD		0x01
#define JS_LT_ID_WGP		0x06

/*
 * Timing sequences for magic commands.
 */

static int js_lt_seq_digital[] __initdata = { 6000, 11000, 7000, 9000, 0 };
static int js_lt_seq_analog[] __initdata = { 2000, 3000, 0 };

/*
 * Port probing variables.
 */

static int js_lt_port_list[] __initdata = { 0x201, 0 };
static struct js_port* js_lt_port __initdata = NULL;

/*
 * Device names.
 */

#define JS_LT_MAX_ID		7

static char *js_lt_names[] = {"WingMan Extreme Digital", "ThunderPad Digital", "Sidecar", "CyberMan 2",
				"WingMan Interceptor", "WingMan Formula", "WingMan GamePad", 
				  "Unknown Device %#x"};

/*
 * Hat to axis conversion arrays.
 */

static struct {
	int x;
	int y;
} js_lt_hat_to_axis[] = {{ 0, 0}, { 0,-1}, { 1,-1}, { 1, 0}, { 1, 1}, { 0, 1}, {-1, 1}, {-1, 0}, {-1,-1}};

/*
 * Per-port information.
 */

struct js_lt_info {
	int  io;
	int  length[2];
	int  ret[2];
	int  idx[2];
	unsigned char id[2];
	char buttons[2];
	char axes10[2];
	char axes8[2];
	char pad[2];
	char hats[2];
	char name[2][JS_LT_MAX_NAME_LENGTH];
	unsigned char data[2][JS_LT_MAX_LENGTH];
};

/*
 * js_lt_read_packet() reads a Logitech ADI packet.
 */

static void js_lt_read_packet(struct js_lt_info *info)
{
	unsigned char u, v, w;
	int t[2];
	unsigned long flags;
	int i;

	int strobe = (js_io_speed * JS_LT_MAX_STROBE) >> 10;
	t[0] = t[1] = (js_io_speed * JS_LT_MAX_START) >> 10;
	i = info->ret[0] = info->ret[1] = info->idx[0] = info->idx[1] = 0;

	__save_flags(flags);
	__cli();

	outb(0xff, info->io);
	u = inb(info->io);

	do {
		t[0]--; t[1]--;
		v = inb(info->io);
		for (i = 0, w = u ^ v; i < 2 && w; i++, w >>= 2)
			if (w & 0x30) {
				if ((w & 0x30) < 0x30 && info->ret[i] < JS_LT_MAX_LENGTH && t[i] > 0) {
					info->data[i][info->ret[i]++] = w;
					t[i] = strobe;
					u = v;
				} else t[i] = 0;
			}
	} while (t[0] > 0 || t[1] > 0);

	__restore_flags(flags);

	info->ret[0]--;
	info->ret[1]--;

	if (info->ret[0] > 0 && info->ret[1] > 0 &&
		~info->data[0][0] & 0x20 && info->data[1][0] & 0x20) {

		for (i = 1; i <= info->ret[1]; i++)
			info->data[0][info->ret[1] + i] = info->data[1][i];
	
		info->ret[0] += info->ret[1];
		info->ret[1] = -1;
	}

	return;
}

/*
 * js_lt_get_bits() gathers bits from the data packet.
 */

static inline int js_lt_get_bits(struct js_lt_info *info, int device, int count)
{
	int bits = 0;
	int i;
	if ((info->idx[device] += count) > info->ret[device]) return 0;
	for (i = 0; i < count; i++) bits |= ((info->data[device][info->idx[device] - i] >> 5) & 1) << i; 
	return bits;
}

/*
 * js_lt_read() reads and analyzes Logitech joystick data.
 */

static int js_lt_read(void *xinfo, int **axes, int **buttons)
{
	struct js_lt_info *info = xinfo;
	int i, j, k, l, t;

	js_lt_read_packet(info);

	for (i = 0; i < 2; i++) {

		if (!info->length[i]) continue;
		if (info->length[i] != info->ret[i]) return -1;

		if (info->id[i] != (js_lt_get_bits(info, i, 4) 
					| (js_lt_get_bits(info, i, 4) << 4))) return -1;
		k = l = 0;

		for (j = 0; j < info->axes10[i]; j++) 
			axes[i][k++] = js_lt_get_bits(info, i, 10);

		for (j = 0; j < info->axes8[i]; j++) 
			axes[i][k++] = js_lt_get_bits(info, i, 8);

		for (j = 0; j <= (info->buttons[i] - 1) >> 5; j++) buttons[i][j] = 0;

		for (j = 0; j < info->buttons[i] && j < 63; j++) {
			if (j == info->pad[i]) {
				t = js_lt_get_bits(info, i, 4);
				axes[i][k++] = ((t >> 2) & 1) - ( t       & 1);
				axes[i][k++] = ((t >> 1) & 1) - ((t >> 3) & 1);
			}
			buttons[i][l >> 5] |= js_lt_get_bits(info, i, 1) << (l & 0x1f);
			l++;
		}

		for (j = 0; j < info->hats[i]; j++) {
			if((t = js_lt_get_bits(info, i, 4)) > 8) return -1;
			axes[i][k++] = js_lt_hat_to_axis[t].x;
			axes[i][k++] = js_lt_hat_to_axis[t].y;
		}

		if (info->buttons[i] > 63)
			for (j = 63; j < info->buttons[i]; j++) {
				buttons[i][l >> 5] |= js_lt_get_bits(info, i, 1) << (l & 0x1f);
				l++;
			}

	}

	return 0;
}

/*
 * js_lt_open() is a callback from the file open routine.
 */

static int js_lt_open(struct js_dev *jd)
{
	MOD_INC_USE_COUNT;
	return 0;
}

/*
 * js_lt_close() is a callback from the file release routine.
 */

static int js_lt_close(struct js_dev *jd)
{
	MOD_DEC_USE_COUNT;
	return 0;
}

/*
 * js_lt_trigger_sequence() sends a trigger & delay sequence
 * to reset/initialize a Logitech joystick.
 */

static void __init js_lt_trigger_sequence(int io, int *seq)
{
	unsigned long flags;

	__save_flags(flags);
	__cli();

	while (*seq) {
		outb(0xff,io);
		udelay(*seq++);
	}
	outb(0xff,io);

	__restore_flags(flags);
}

/*
 * js_lt_init_corr() initializes the correction values for
 * Logitech joysticks.
 */

static void __init js_lt_init_corr(int naxes10, int naxes8, int naxes1, int *axes, struct js_corr *corr)
{
	int j;
	
	if (!naxes8 && (naxes10 == 3)) axes[2] = 512;	/* Throttle fixup */
	if (!naxes10 && (naxes8 == 3)) axes[2] = 128;

	for (j = 0; j < naxes10; j++) {
		corr[j].type = JS_CORR_BROKEN;
		corr[j].prec = 4;
		corr[j].coef[0] = axes[j] - 8;
		corr[j].coef[1] = axes[j] + 8;
		corr[j].coef[2] = (1 << 29) / (256 - 64);
		corr[j].coef[3] = (1 << 29) / (256 - 64);
	}

	for (; j < naxes8 + naxes10; j++) {
		corr[j].type = JS_CORR_BROKEN;
		corr[j].prec = 1;
		corr[j].coef[0] = axes[j] - 2;
		corr[j].coef[1] = axes[j] + 2;
		corr[j].coef[2] = (1 << 29) / (64 - 16);
		corr[j].coef[3] = (1 << 29) / (64 - 16);
	}

	for (; j < naxes1 + naxes8 + naxes10; j++) {
		corr[j].type = JS_CORR_BROKEN;
		corr[j].prec = 0;
		corr[j].coef[0] = 0;
		corr[j].coef[1] = 0;
		corr[j].coef[2] = (1 << 29);
		corr[j].coef[3] = (1 << 29);
	}

}

/*
 * js_lt_probe() probes for Logitech type joysticks.
 */

static struct js_port __init *js_lt_probe(int io, struct js_port *port)
{
	struct js_lt_info info;
	char name[32];
	int i, j, t;

	if (check_region(io, 1)) return port;

	js_lt_trigger_sequence(io, js_lt_seq_analog);
	udelay(JS_LT_SYNC_DELAY);
	js_lt_trigger_sequence(io, js_lt_seq_digital);
	udelay(JS_LT_STATUS_DELAY);

	memset(&info, 0, sizeof(struct js_lt_info));

	info.length[0] = info.length[1] = JS_LT_MAX_LENGTH;

	info.io = io;
	js_lt_read_packet(&info);
	udelay(JS_LT_EXTRA_DELAY);

	info.length[0] = info.length[1] = 0;

	for (i = 0; i < 2; i++) {

		if (info.ret[i] < JS_LT_MIN_ID_LENGTH) continue; /* Minimum ID packet length */

		if (info.ret[i] != (t = js_lt_get_bits(&info, i, 10))) {
			printk(KERN_WARNING "joy-logitech: Wrong ID packet length: reported: %d != read: %d\n",
				t, info.ret[i]); 
			continue;
		}

		info.id[i] = js_lt_get_bits(&info, i, 4) | (js_lt_get_bits(&info, i, 4) << 4);

		t = js_lt_get_bits(&info, i, 4);

		if (t & JS_LT_FLAG_CHAIN) {
			printk(KERN_WARNING "joy-logitech: Daisy-chained devices not supported yet. Ignoring device.\n");
			continue;
		}

		if (t & JS_LT_FLAG_HAT) info.hats[i]++;

		if ((info.length[i] = js_lt_get_bits(&info, i, 10)) >= JS_LT_MAX_LENGTH) {
			printk(KERN_WARNING "joy-logitech: Expected packet length too long (%d).\n",
				info.length[i]);
			continue;
		}

		if (info.length[i] < JS_LT_MIN_LENGTH) {
			printk(KERN_WARNING "joy-logitech: Expected packet length too short (%d).\n",
				info.length[i]);
			continue;
		}

		info.axes8[i] = js_lt_get_bits(&info, i, 4);
		info.buttons[i] = js_lt_get_bits(&info, i, 6);

		if (js_lt_get_bits(&info, i, 6) != 8 && info.hats[i]) {
			printk(KERN_WARNING "joy-logitech: Other than 8-dir POVs not supported yet.\n");
			continue;
		}

		info.buttons[i] += js_lt_get_bits(&info, i, 6);
		info.hats[i] += js_lt_get_bits(&info, i, 4);

		j = js_lt_get_bits(&info, i, 4);

		if (t & JS_LT_FLAG_10BIT) {
			info.axes10[i] = info.axes8[i];
			info.axes8[i] = j;
		}

		t = js_lt_get_bits(&info, i, 4);

		for (j = 0; j < t; j++)
			info.name[i][j] = js_lt_get_bits(&info, i, 8);
		info.name[i][j] = 0;

		switch (info.id[i]) {
			case JS_LT_ID_TPD:
				info.pad[i] = 4;
				info.buttons[i] -= 4;
				break;
			case JS_LT_ID_WGP:
				info.pad[i] = 0;
				info.buttons[i] -= 4;
				break;
			default:
				info.pad[i] = -1;
				break;
		}
	}

	if (!info.length[0] && !info.length[1])
		return port;

	request_region(io, 1, "joystick (logitech)");

	port = js_register_port(port, &info, 2, sizeof(struct js_lt_info), js_lt_read);

	for (i = 0; i < 2; i++)
		if (info.length[i] > 0) {
			sprintf(name, info.id[i] < JS_LT_MAX_ID ? js_lt_names[info.id[i]] : js_lt_names[JS_LT_MAX_ID], info.id[i]); 
			printk(KERN_INFO "js%d: %s [%s] at %#x\n",
				js_register_device(port, i,
					info.axes10[i] + info.axes8[i] + ((info.hats[i] + (info.pad[i] >= 0)) << 1),
					info.buttons[i], name, js_lt_open, js_lt_close), name, info.name[i], io);
		}

	js_lt_read(port->info, port->axes, port->buttons);

	for (i = 0; i < 2; i++)
		if (info.length[i] > 0)
				js_lt_init_corr(info.axes10[i], info.axes8[i],
					((info.pad[i] >= 0) + info.hats[i]) << 1, port->axes[i], port->corr[i]);

	return port;
}

#ifdef MODULE
int init_module(void)
#else
int __init js_lt_init(void)
#endif
{
	int *p;

	for (p = js_lt_port_list; *p; p++) js_lt_port = js_lt_probe(*p, js_lt_port);
	if (js_lt_port) return 0;

#ifdef MODULE
	printk(KERN_WARNING "joy-logitech: no joysticks found\n");
#endif

	return -ENODEV;
}

#ifdef MODULE
void cleanup_module(void)
{
	int i;
	struct js_lt_info *info;

	while (js_lt_port) {
		for (i = 0; i < js_lt_port->ndevs; i++)
			 if (js_lt_port->devs[i])
				js_unregister_device(js_lt_port->devs[i]);
		info = js_lt_port->info;
		release_region(info->io, 1);
		js_lt_port = js_unregister_port(js_lt_port);
	}
}
#endif
