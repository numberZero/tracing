#!/usr/bin/python3

import os
import os.path
import math
import cairo

def draw_grid(ctx, cx, cy):
	ctx.set_source_rgba(0.0, 1.0, 1.0, 0.7)
	for u in range(0, int(cx) + 1, 5):
		ctx.move_to(u, 0)
		ctx.line_to(u, cy)
	for v in range(0, int(cy) + 1, 5):
		ctx.move_to(0, v)
		ctx.line_to(cx, v)
	ctx.stroke()
	ctx.set_source_rgba(0.0, 1.0, 1.0, 0.2)
	for u in range(int(cx) + 1):
		if u % 5 == 0:
			continue
		ctx.move_to(u, 0)
		ctx.line_to(u, cy)
	for v in range(int(cy) + 1):
		if v % 5 == 0:
			continue
		ctx.move_to(0, v)
		ctx.line_to(cx, v)
	ctx.stroke()

SIZE = 2048
surface = cairo.ImageSurface(cairo.FORMAT_ARGB32, SIZE, SIZE)
ctx = cairo.Context(surface)
ctx.set_source_rgba(0.0, 0.0, 0.0, 1.0)
ctx.paint()

ctx.save()
ctx.scale(2048, 2048)
ctx.set_operator(cairo.OPERATOR_ADD)

pat = cairo.LinearGradient(0.0, 0.0, 0.0, 1.0)
pat.add_color_stop_rgba(0, 0, 1, 0, 1)
pat.add_color_stop_rgba(1, 0, 0, 0, 1)
ctx.set_source(pat)
ctx.paint()

pat = cairo.LinearGradient(0.0, 0.0, 1.0, 0.0)
pat.add_color_stop_rgba(1, 1, 0, 0, 1)
pat.add_color_stop_rgba(0, 0, 0, 0, 1)
ctx.set_source(pat)
ctx.paint()

ctx.restore()

ctx.scale(102.4, 102.4)
ctx.set_line_width(0.05)
draw_grid(ctx, 20, 20)
ctx.set_source_rgba(1.0, 1.0, 1.0, 1.0)
ctx.move_to(10, 0)
ctx.line_to(10, 20)
ctx.move_to(0, 10)
ctx.line_to(20, 10)
ctx.stroke()
ctx.select_font_face("DejaVu Sans", cairo.FONT_SLANT_NORMAL, cairo.FONT_WEIGHT_NORMAL)
ctx.set_font_size(0.3)
for u in range(20):
	ctx.move_to(u + 0.1, 9.9)
	ctx.show_text(f'{u/10-1.0:.1f}')
for v in range(20):
	if v == 10:
		continue
	ctx.move_to(10.1, 19.9 - v)
	ctx.show_text(f'{v/10-1.0:.1f}')

os.chdir(os.path.dirname(__file__))
surface.write_to_png('grid.png')
