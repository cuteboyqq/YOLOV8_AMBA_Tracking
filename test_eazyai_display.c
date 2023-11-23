/*******************************************************************************
 * test_eazyai_display.c
 *
 * History:
 *  2022/12/20 - [Wei Liu] created
 *
 *
 * Copyright (c) 2022 Ambarella International LP
 *
 * This file and its contents ( "Software" ) are protected by intellectual
 * property rights including, without limitation, U.S. and/or foreign
 * copyrights. This Software is also the confidential and proprietary
 * information of Ambarella International LP and its licensors. You may not use, reproduce,
 * disclose, distribute, modify, or otherwise prepare derivative works of this
 * Software or any portion thereof except pursuant to a signed license agreement
 * or nondisclosure agreement with Ambarella International LP or its authorized affiliates.
 * In the absence of such an agreement, you agree to promptly notify and return
 * this Software to Ambarella International LP.
 *
 * This file includes sample code and is only for internal testing and evaluation.  If you
 * distribute this sample code (whether in source, object, or binary code form), it will be
 * without any warranty or indemnity protection from Ambarella International LP or its affiliates.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF NON-INFRINGEMENT,
 * MERCHANTABILITY, AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL AMBARELLA INTERNATIONAL LP OR ITS AFFILIATES BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; COMPUTER FAILURE OR MALFUNCTION; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <signal.h>
#include <core/eazyai_core.h>
#include <utils/ea_display.h>

EA_LOG_DECLARE_LOCAL(EA_LOG_LEVEL_NOTICE);

#define MAX_STR_LEN 256

static int exit_flag = 0;

enum {
	DRAW_MODE_VOUT = 0,
	DRAW_MODE_STREAM = 1,
};

enum scanf_next_action_opt {
	SCANF_NEXT_ACTION_BREAK = 0,
	SCANF_NEXT_ACTION_CONTINUE = 1,
};

/**********************************************************
 * Just check the case of entering a parameter via scanf(),
 * and if scanf() returns an error, function_rval is -1
 *
 **********************************************************/

#ifndef DISP_SCANF_CHECK
#define DISP_SCANF_CHECK(scanf_rval, err_str, function_rval, next_action)	\
	if ((scanf_rval) != 1) { \
		printf("Scanf Rval: %s !\n", err_str);	\
		(function_rval) = -1;	\
		if ((next_action) == SCANF_NEXT_ACTION_BREAK) {	\
			break;	\
		} else if ((next_action) == SCANF_NEXT_ACTION_CONTINUE) {	\
			continue;	\
		} else {	\
			printf("%s:%d, check next_action error !\n", __func__, __LINE__);	\
			return -1;	\
		}	\
	}
#endif

typedef struct display_ctx_s {
	ea_display_t *display;

	int draw_mode;
	int stream_id;
} display_ctx_t;

static display_ctx_t ctx;

typedef struct box_input_params {
	char text[MAX_STR_LEN];
	float x_start;
	float y_start;
	float w;
	float h;
} box_input_params_t;

typedef struct line_input_params {
	float start_x;
	float start_y;
	float end_x;
	float end_y;
	ea_display_line_params_t params;
} line_input_params_t;

typedef struct circle_input_params {
	float center_x;
	float center_y;
	float radius;
	ea_display_circle_params_t params;
} circle_input_params_t;

static void usage(void)
{
	printf("test_eazyai_display:\n\n");
	printf("\t test_eazyai_display\n"
		"\t Main Menu\n"
		"\t\tv - Display On Vout\n"
		"\t\ts - Display On Stream\n"
		"\t\tq - Quit\n");
	printf("\n");
}

static void show_main_menu(void)
{
	printf("\n|---------------------------------|\n");
	printf("| Main Menu                       |\n");
	printf("| v - Display On Vout             |\n");
	printf("| s - Display On Stream           |\n");
	printf("| q - Quit                        |\n");
	printf("|---------------------------------|\n");
}

static int show_display_menu(int draw_mode)
{
	printf("\n|-----------------------------------------------------------|\n");
	if (draw_mode == DRAW_MODE_VOUT) {
		printf("|                      - Draw On Vout -                     |\n");
	} else if (draw_mode == DRAW_MODE_STREAM) {
		printf("|                      - Draw On Stream -                   |\n");
	}
	printf("|-----------------------------------------------------------|\n");
	printf("|  1 - Draw bbox (stream blur)                              |\n");
	printf("|  2 - Draw textbox                                         |\n");
	printf("|  3 - Draw line                                            |\n");
	printf("|  4 - Draw circle                                          |\n");
	printf("|  5 - Display refresh                                      |\n");
	printf("|  q - Back to Main Menu                                    |\n");
	printf("|-----------------------------------------------------------|\n");

	return 0;
}

static int input_box_attr(box_input_params_t *input)
{
	int rval = EA_SUCCESS;
	int scanf_rval = 0;
	int tmp_int = 0;
	float tmp_float = 0.0f;
	char tmp_str[MAX_STR_LEN];
	ea_16_colors_t tmp_color = 0;
	ea_title_pos_type_t tmp_pos = 0;

	do {
		printf("\n** Input Function Attribution (Reused by function bbox & textbox & stream blur) **\n");
		printf("\n==> Input start x: ");
		scanf_rval = scanf("%f", &tmp_float);
		DISP_SCANF_CHECK(scanf_rval, \
			"Input start x of box ERROR", rval, SCANF_NEXT_ACTION_BREAK);
		input->x_start = tmp_float;

		printf("\n==> Input start y: ");
		scanf_rval = scanf("%f", &tmp_float);
		DISP_SCANF_CHECK(scanf_rval, \
			"Input start y of box ERROR", rval, SCANF_NEXT_ACTION_BREAK);
		input->y_start = tmp_float;

		printf("\n==> Input width: ");
		scanf_rval = scanf("%f", &tmp_float);
		DISP_SCANF_CHECK(scanf_rval, \
			"Input box witdth ERROR", rval, SCANF_NEXT_ACTION_BREAK);
		input->w = tmp_float;

		printf("\n==> Input height: ");
		scanf_rval = scanf("%f", &tmp_float);
		DISP_SCANF_CHECK(scanf_rval, \
			"Input box height ERROR", rval, SCANF_NEXT_ACTION_BREAK);
		input->h = tmp_float;

		printf("\n[Notice] set '-1' to draw solid box.");
		printf("\n==> Input border thickness(useless in blur mode): ");
		scanf_rval = scanf("%d", &tmp_int);
		DISP_SCANF_CHECK(scanf_rval, \
			"Input border thickness ERROR", rval, SCANF_NEXT_ACTION_BREAK);
		ea_display_obj_params(ctx.display)->border_thickness = tmp_int;

		printf("\n[Notice] 0: aqua, 1: black, 2: blue, 3: fuchsia, 4: gray 5: green 6: lime, 7: maroon, ");
		printf("8: navy, 9: olive, 10: purple, 11: red, 12: silver, 13: teal, 14: white, 15: yellow.");
		printf("\n==> Input box color(if solid box, will also be bg color, useless in blur mode): ");
		scanf_rval = scanf("%d", (int *)&tmp_color);
		DISP_SCANF_CHECK(scanf_rval, \
			"Input box color ERROR", rval, SCANF_NEXT_ACTION_BREAK);
		if (tmp_color < 0 || tmp_color > 15) {
			printf("[Warning] [%d] out of range, '0: aqua' as default.\n", tmp_color);
			ea_display_obj_params(ctx.display)->box_color = EA_16_COLORS_AQUA;
		} else {
			ea_display_obj_params(ctx.display)->box_color = tmp_color;
		}

		if (ea_display_obj_params(ctx.display)->border_thickness != -1) {
			printf("\n[Notice] The background transparency of the box. min 0: transparent, max 255: non-transparent.");
			printf("\n==> Input box background transparency(useless in blur mode): ");
			scanf_rval = scanf("%d", &tmp_int);
			DISP_SCANF_CHECK(scanf_rval, \
				"Input box background transparency ERROR", rval, SCANF_NEXT_ACTION_BREAK);
			if (tmp_int < 0 || tmp_int > 255) {
				printf("[Warning] [%d] out of range, non-transparent as default.\n", tmp_int);
				ea_display_obj_params(ctx.display)->box_background_transparency = 255;
			} else {
				ea_display_obj_params(ctx.display)->box_background_transparency = tmp_int;
			}
		}

		if (ctx.draw_mode == DRAW_MODE_STREAM) {
			printf("\n[Notice] Just for blur mode, valid range is '0~2', set '-1' to disable blur mode.");
			printf("\n==> Input blur strength: ");
			scanf_rval = scanf("%d", &tmp_int);
			DISP_SCANF_CHECK(scanf_rval, \
				"Input blur strength ERROR", rval, SCANF_NEXT_ACTION_BREAK);
			if (tmp_int < -1 || tmp_int > 2) {
				printf("[Warning] [%d] out of range, strength '2' as default.\n", tmp_int);
				ea_display_obj_params(ctx.display)->blur_strength = 2;
			} else {
				ea_display_obj_params(ctx.display)->blur_strength = tmp_int;
			}
		}

		printf("\n[Notice] 0: auto, 1: above, 2: below, 3: left, 4: right.");
		printf("\n[Warning] If textbox, only '0:auto' is valid.");
		printf("\n==> Input title position(useless in blur mode): ");
		scanf_rval = scanf("%d", (int *)&tmp_pos);
		DISP_SCANF_CHECK(scanf_rval, \
			"Input title position ERROR", rval, SCANF_NEXT_ACTION_BREAK);
		if (tmp_pos < 0 || tmp_pos > 4) {
			printf("[Warning] [%d] out of range, '0: auto' as default.\n", tmp_pos);
			ea_display_obj_params(ctx.display)->title_pos = EA_TITLE_POS_AUTO;
		} else {
			ea_display_obj_params(ctx.display)->title_pos = tmp_pos;
		}

		printf("\n==> Input font size(useless in blur mode): ");
		scanf_rval = scanf("%d", &tmp_int);
		DISP_SCANF_CHECK(scanf_rval, \
			"Input font size ERROR", rval, SCANF_NEXT_ACTION_BREAK);
		ea_display_obj_params(ctx.display)->font_size = tmp_int;

		printf("\n[Notice] 0: aqua, 1: black, 2: blue, 3: fuchsia, 4: gray 5: green 6: lime, 7: maroon, ");
		printf("8: navy, 9: olive, 10: purple, 11:red, 12: silver, 13: teal, 14: white, 15: yellow.");
		printf("\n==> Input text color(useless in blur mode): ");
		scanf_rval = scanf("%d", (int *)&tmp_color);
		DISP_SCANF_CHECK(scanf_rval, \
			"Input text color ERROR", rval, SCANF_NEXT_ACTION_BREAK);
		if (tmp_color < 0 || tmp_color > 15) {
			printf("[Warning] [%d] out of range, '0: aqua' as default.\n", tmp_color);
			ea_display_obj_params(ctx.display)->text_color = EA_16_COLORS_AQUA;
		} else {
			ea_display_obj_params(ctx.display)->text_color = tmp_color;
		}

		printf("\n[Notice] 0: aqua, 1: black, 2: blue, 3: fuchsia, 4: gray 5: green 6: lime, 7: maroon, ");
		printf("8: navy, 9: olive, 10: purple, 11:red, 12: silver, 13: teal, 14: white, 15: yellow.");
		printf("\n==> Input text background color(useless in blur mode): ");
		scanf_rval = scanf("%d", (int *)&tmp_color);
		DISP_SCANF_CHECK(scanf_rval, \
			"Input text background color ERROR", rval, SCANF_NEXT_ACTION_BREAK);
		if (tmp_color < 0 || tmp_color > 15) {
			printf("[Warning] [%d] out of range, '0: aqua' as default.\n", tmp_color);
			ea_display_obj_params(ctx.display)->text_background_color = EA_16_COLORS_AQUA;
		} else {
			ea_display_obj_params(ctx.display)->text_background_color = tmp_color;
		}

		printf("\n[Notice] The background transparency of the title or the text.");
		printf("min 0: transparent, max 255: non-transparent.");
		printf("\n==> Input text background transparency(useless in blur mode): ");
		scanf_rval = scanf("%d", &tmp_int);
		DISP_SCANF_CHECK(scanf_rval, \
			"Input text background transparency ERROR", rval, SCANF_NEXT_ACTION_BREAK);
		if (tmp_int < 0 || tmp_int > 255) {
			printf("[Warning] [%d] out of range, non-transparent as default.\n", tmp_int);
			ea_display_obj_params(ctx.display)->text_background_transparency = 255;
		} else {
			ea_display_obj_params(ctx.display)->text_background_transparency = tmp_int;
		}

		printf("\n==> Input text string(useless in blur mode): ");
		scanf_rval = scanf("%s", tmp_str);
		DISP_SCANF_CHECK(scanf_rval, \
			"Input text string ERROR", rval, SCANF_NEXT_ACTION_BREAK);
		if (strlen(tmp_str) + 1 > MAX_STR_LEN) {
			printf("ERROR! The length of text string is out of range.");
			rval = -1;
			break;
		}
		snprintf(input->text, sizeof(tmp_str), "%s", tmp_str);
		printf("\n** Execute Refresh **\n");
	} while (0);

	return rval;
}

static int input_line_attr(line_input_params_t *input)
{
	int rval = EA_SUCCESS;
	int scanf_rval = 0;
	int tmp_int = 0;
	float tmp_float = 0.0f;

	do {
		printf("\n** Input Function Attribution **\n");
		printf("\n==> Input start x: ");
		scanf_rval = scanf("%f", &tmp_float);
		DISP_SCANF_CHECK(scanf_rval, \
			"Input start x of line ERROR", rval, SCANF_NEXT_ACTION_BREAK);
		input->start_x = tmp_float;

		printf("\n==> Input start y: ");
		scanf_rval = scanf("%f", &tmp_float);
		DISP_SCANF_CHECK(scanf_rval, \
			"Input start y of line ERROR", rval, SCANF_NEXT_ACTION_BREAK);
		input->start_y = tmp_float;

		printf("\n==> Input end x: ");
		scanf_rval = scanf("%f", &tmp_float);
		DISP_SCANF_CHECK(scanf_rval, \
			"Input end x of line ERROR", rval, SCANF_NEXT_ACTION_BREAK);
		input->end_x = tmp_float;

		printf("\n==> Input end y: ");
		scanf_rval = scanf("%f", &tmp_float);
		DISP_SCANF_CHECK(scanf_rval, \
			"Input end y of line ERROR", rval, SCANF_NEXT_ACTION_BREAK);
		input->end_y = tmp_float;

		printf("\n==> Input line thickness: ");
		scanf_rval = scanf("%d", &tmp_int);
		DISP_SCANF_CHECK(scanf_rval, \
			"Input line thickness ERROR", rval, SCANF_NEXT_ACTION_BREAK);
		input->params.thickness = tmp_int;

		printf("\n[Notice] 0: aqua, 1: black, 2: blue, 3: fuchsia, 4: gray 5: green 6: lime, 7: maroon, ");
		printf("8: navy, 9: olive, 10: purple, 11:red, 12: silver, 13: teal, 14: white, 15: yellow.");
		printf("\n==> Input line color: ");
		scanf_rval = scanf("%d", &tmp_int);
		DISP_SCANF_CHECK(scanf_rval, \
			"Input line color ERROR", rval, SCANF_NEXT_ACTION_BREAK);
		if (tmp_int < 0 || tmp_int > 15) {
			printf("[Warning] [%d] out of range, '0: aqua' as default.\n", tmp_int);
			input->params.color = 0;
		} else {
			input->params.color = tmp_int;
		}

		printf("\n** Execute Refresh **\n");
	} while (0);

	return rval;
}

static int input_circle_attr(circle_input_params_t *input)
{
	int rval = EA_SUCCESS;
	int scanf_rval = 0;
	int tmp_int = 0;
	float tmp_float = 0.0f;

	do {
		printf("\n** Input Function Attribution **\n");
		printf("\n==> Input center x: ");
		scanf_rval = scanf("%f", &tmp_float);
		DISP_SCANF_CHECK(scanf_rval, \
			"Input circle center x ERROR", rval, SCANF_NEXT_ACTION_BREAK);
		input->center_x = tmp_float;

		printf("\n==> Input center y: ");
		scanf_rval = scanf("%f", &tmp_float);
		DISP_SCANF_CHECK(scanf_rval, \
			"Input circle center y ERROR", rval, SCANF_NEXT_ACTION_BREAK);
		input->center_y = tmp_float;

		printf("\n==> Input radius: ");
		scanf_rval = scanf("%f", &tmp_float);
		DISP_SCANF_CHECK(scanf_rval, \
			"Input circle radius ERROR", rval, SCANF_NEXT_ACTION_BREAK);
		input->radius = tmp_float;

		printf("\n[Notice] set '-1' to draw solid circle.");
		printf("\n==> Input thickness: ");
		scanf_rval = scanf("%d", &tmp_int);
		DISP_SCANF_CHECK(scanf_rval, \
			"Input circle thickness ERROR", rval, SCANF_NEXT_ACTION_BREAK);
		input->params.thickness = tmp_int;

		printf("\n[Notice] 0: aqua, 1: black, 2: blue, 3: fuchsia, 4: gray 5: green 6: lime, 7: maroon, ");
		printf("8: navy, 9: olive, 10: purple, 11:red, 12: silver, 13: teal, 14: white, 15: yellow.");
		printf("\n==> Input cirlce line color(if solid circle, will also be bg color): ");
		scanf_rval = scanf("%d", &tmp_int);
		DISP_SCANF_CHECK(scanf_rval, \
			"Input circle color ERROR", rval, SCANF_NEXT_ACTION_BREAK);
		if (tmp_int < 0 || tmp_int > 15) {
			printf("[Warning] [%d] out of range, '0: aqua' as default.\n", tmp_int);
			input->params.color = 0;
		} else {
			input->params.color = tmp_int;
		}

		if (input->params.thickness != -1) {
			printf("\n[Notice] 0: aqua, 1: black, 2: blue, 3: fuchsia, 4: gray 5: green 6: lime, 7: maroon, ");
			printf("8: navy, 9: olive, 10: purple, 11:red, 12: silver, 13: teal, 14: white, 15: yellow.");
			printf("\n==> Input background color: ");
			scanf_rval = scanf("%d", &tmp_int);
			DISP_SCANF_CHECK(scanf_rval, \
				"Input circle background color ERROR", rval, SCANF_NEXT_ACTION_BREAK);
			if (tmp_int < 0 || tmp_int > 15) {
				printf("[Warning] [%d] out of range, '0: aqua' as default.\n", tmp_int);
				input->params.bg_color = 0;
			} else {
				input->params.bg_color = tmp_int;
			}

			printf("\n[Notice] Background transparency. min 0: transparent, max 255: non-transparent.");
			printf("\n==> Input background transparency: ");
			scanf_rval = scanf("%d", &tmp_int);
			DISP_SCANF_CHECK(scanf_rval, \
				"Input circle background bg_transparency ERROR", rval, SCANF_NEXT_ACTION_BREAK);
			if (tmp_int < 0 || tmp_int > 255) {
				printf("[Warning] [%d] out of range, non-transparent as default.\n", tmp_int);
				input->params.bg_transparency = 255;
			} else {
				input->params.bg_transparency = tmp_int;
			}
		}
		printf("\n** Execute Refresh **\n");
	} while (0);

	return rval;
}

static void set_obj_win_attr(display_ctx_t *ctx)
{
	do {
		printf("\n** OBJ Window Attribution **\n");
		printf("\n[Notice] OBJ window size is [%0.1f]x[%0.1f].", \
			ea_display_obj_params(ctx->display)->dis_win_w,
			ea_display_obj_params(ctx->display)->dis_win_h);

		ea_display_obj_params(ctx->display)->obj_win_w = \
			ea_display_obj_params(ctx->display)->dis_win_w;

		ea_display_obj_params(ctx->display)->obj_win_h = \
			ea_display_obj_params(ctx->display)->dis_win_h;
	} while (0);
}

static int display_init(display_ctx_t *ctx)
{
	int rval = EA_SUCCESS;
	int features = 0;

	do {
		features = EA_ENV_ENABLE_IAV;

		if (ctx->draw_mode ==  DRAW_MODE_VOUT) {
			features |= EA_ENV_ENABLE_OSD_VOUT;
		} else if (ctx->draw_mode == DRAW_MODE_STREAM) {
			features |= EA_ENV_ENABLE_OSD_STREAM;
		}

		RVAL_OK(ea_env_open(features));

		if (ctx->draw_mode== DRAW_MODE_VOUT) {
			ctx->display = ea_display_new(EA_DISPLAY_VOUT, EA_DISPLAY_ANALOG_VOUT, EA_DISPLAY_BBOX_TEXTBOX, NULL);
		} else if (ctx->draw_mode == DRAW_MODE_STREAM) {
			ctx->display = ea_display_new(EA_DISPLAY_STREAM, ctx->stream_id, EA_DISPLAY_BBOX_TEXTBOX, NULL);
		}

		set_obj_win_attr(ctx);
	} while (0);

	return rval;
}

static void display_deinit(void)
{
	if (ctx.display) {
		ea_display_free(ctx.display);
		ctx.display = NULL;
	}

	ea_env_close();
}

static void stop_display(int sig_num)
{
	exit_flag = 1;
	display_deinit();
	exit(0);
}

static int display_config()
{
	int rval = EA_SUCCESS;
	int back2main = 0;
	int scanf_rval = 0;
	char opt, input[16];
	uint32_t dsp_pts = 0;

	box_input_params_t bbox_input;
	box_input_params_t textbox_input;
	line_input_params_t line_input;
	circle_input_params_t circle_input;

	do {
		memset(&bbox_input, 0, sizeof(box_input_params_t));
		memset(&textbox_input, 0, sizeof(box_input_params_t));
		memset(&line_input, 0, sizeof(line_input_params_t));
		memset(&circle_input, 0, sizeof(circle_input_params_t));

		RVAL_OK(display_init(&ctx));

		while (back2main == 0) {
			show_display_menu(ctx.draw_mode);
			printf("\n** Sub Menu Choice **\n");
			printf("\n==> Input your choice: ");
			scanf_rval = scanf("%s", input);
			if (scanf_rval != 1) {
				printf("Input your choice ERROR.");
				continue;
			}
			opt = tolower(input[0]);
			switch (opt) {
			case '1':
				RVAL_OK(input_box_attr(&bbox_input));
				RVAL_OK(ea_display_set_bbox(ctx.display, bbox_input.text, \
					bbox_input.x_start, bbox_input.y_start, bbox_input.w, bbox_input.h));
				break;
			case '2':
				RVAL_OK(input_box_attr(&textbox_input));
				RVAL_OK(ea_display_set_textbox(ctx.display, textbox_input.text, \
					textbox_input.x_start, textbox_input.y_start, textbox_input.w, textbox_input.h));
				break;
			case '3':
				RVAL_OK(input_line_attr(&line_input));
				RVAL_OK(ea_display_set_line(ctx.display, line_input.start_x, line_input.start_y, \
					line_input.end_x, line_input.end_y, &line_input.params));
				break;
			case '4':
				RVAL_OK(input_circle_attr(&circle_input));
				RVAL_OK(ea_display_set_circle(ctx.display, \
					circle_input.center_x, circle_input.center_y, circle_input.radius, &circle_input.params));
				break;
			case '5':
				RVAL_OK(ea_display_refresh(ctx.display, (void *)(unsigned long)dsp_pts));
				printf("\n** Display Refreshed **\n");
				break;
			case 'q':
				display_deinit();
				back2main = 1;
				break;
			default:
				printf("unknown option\n");
				break;
			}
		}
	} while (0);

	if (rval == EA_FAIL) {
		display_deinit();
	}

	return rval;
}

int main(int argc, char **argv)
{
	int rval = EA_SUCCESS;
	int scanf_rval = 0;
	char opt, input[16];

	do {
		if (argc > 1) {
			usage();
			rval = -1;
			break;
		}

		signal(SIGINT, stop_display);
		signal(SIGTERM, stop_display);
		signal(SIGQUIT, stop_display);

		memset(&ctx, 0, sizeof(display_ctx_t));

		while (exit_flag == 0) {
			show_main_menu();
			printf("\n** Main Menu Choice **\n");
			printf("\n==> Input your choice: ");
			scanf_rval = scanf("%s", input);
			if (scanf_rval != 1) {
				printf("Input your choice ERROR.");
				continue;
			}
			opt = tolower(input[0]);
			switch (opt) {
			case 'v':
				ctx.draw_mode = DRAW_MODE_VOUT;
				display_config();
				break;
			case 's':
				ctx.draw_mode = DRAW_MODE_STREAM;
				display_config();
				break;
			case 'q':
				exit_flag = 1;
				break;
			default:
				printf("unknown option %d.", opt);
				break;
			}
		}
	} while(0);

	return rval;
}
