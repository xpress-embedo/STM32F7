/*
 * rgb_mixer.c
 *
 *  Created on: 01-Nov-2022
 *      Author: xpress_embedo
 */
#include "lvgl/lvgl.h"

typedef enum _SliderType_e
{
  SLIDER_TYPE_RED = 0,
  SLIDER_TYPE_GREEN,
  SLIDER_TYPE_BLUE,
} SliderType_e;

typedef struct _RGB_Mixer_s
{
  SliderType_e  slider_type;
  lv_obj_t*     label;
} RGB_Mixer_s;

/*---------------------------Private Variables--------------------------------*/
static RGB_Mixer_s red, green, blue;
static lv_obj_t *rectangle;
static lv_style_t style;

/*--------------------------Private Function Prototypes-----------------------*/
static void slider_callback( lv_event_t *e );

/*---------------------------Public Function Definitions----------------------*/
void rgb_mixer_created_ui( void )
{
  lv_obj_t *act_scr = lv_scr_act();                     // Get the active screen object

  // intialize the styles
  lv_style_init(&style);
  // we have to enable other font sizes in menuconfig
  lv_style_set_text_font(&style, &lv_font_montserrat_32); 

  // RED, Green and Blue Slider Configuration
  lv_obj_t *slider_r = lv_slider_create( act_scr );     // create a red slider base object
  lv_obj_t *slider_g = lv_slider_create( act_scr );     // create a green slider base object
  lv_obj_t *slider_b = lv_slider_create( act_scr );     // create a blue slider base object

  // Setting Sliders Width
  lv_obj_set_width( slider_r, LV_PCT(80) );
  lv_obj_set_width( slider_g, LV_PCT(80) );
  lv_obj_set_width( slider_b, LV_PCT(80) );

  // Setting Sliders Height
  lv_obj_set_height( slider_r, LV_PCT(4) );
  lv_obj_set_height( slider_g, LV_PCT(4) );
  lv_obj_set_height( slider_b, LV_PCT(4) );
  
  // Align Sliders with Each Other
  lv_obj_align( slider_r, LV_ALIGN_TOP_MID, 0u, LV_PCT(20) );
  lv_obj_align_to( slider_g, slider_r, LV_ALIGN_TOP_MID, 0u, 70u );
  lv_obj_align_to( slider_b, slider_g, LV_ALIGN_TOP_MID, 0u, 70u );

  // set slider range also (by default it is 0 to 100 but we want till 255)
  lv_slider_set_range( slider_r, 0, 255 );
  lv_slider_set_range( slider_g, 0, 255 );
  lv_slider_set_range( slider_b, 0, 255 );
  
  // Coloring Sliders, Slider has three parts Main, Indicator and Knob
  // apply red color to the indicator part
  lv_obj_set_style_bg_color( slider_r, lv_palette_main(LV_PALETTE_RED), LV_PART_INDICATOR );
  // apply red color to the knob part
  lv_obj_set_style_bg_color( slider_r, lv_palette_main(LV_PALETTE_RED), LV_PART_KNOB );

  // apply green color to the indicator part
  lv_obj_set_style_bg_color( slider_g, lv_palette_main(LV_PALETTE_GREEN), LV_PART_INDICATOR );
  // apply green color to the knob part
  lv_obj_set_style_bg_color( slider_g, lv_palette_main(LV_PALETTE_GREEN), LV_PART_KNOB );

  // apply blue color to the indicator part
  lv_obj_set_style_bg_color( slider_b, lv_palette_main(LV_PALETTE_BLUE), LV_PART_INDICATOR );
  // apply blue color to the knob part
  lv_obj_set_style_bg_color( slider_b, lv_palette_main(LV_PALETTE_BLUE), LV_PART_KNOB );

  rectangle = lv_obj_create(act_scr);                   // Creates a base object Rectangle to display color
  lv_obj_set_size( rectangle, LV_PCT(93), LV_PCT(33) );
  lv_obj_align_to( rectangle, slider_b, LV_ALIGN_TOP_MID, 0u, 50u );
  lv_obj_set_style_border_color( rectangle, lv_color_black(), LV_PART_MAIN );   // add black border to rectangle
  lv_obj_set_style_border_width( rectangle, 2, LV_PART_MAIN );                  // increase the width of the border by 2px
  lv_obj_set_style_bg_color( rectangle, lv_color_make( 0, 0, 0), LV_PART_MAIN); // all sliders are at zero, so background color should be black

  // Create Main Heading Label
  lv_obj_t *heading = lv_label_create(act_scr);
  lv_label_set_text( heading, "RGB Mixer");
  lv_obj_align( heading, LV_ALIGN_TOP_MID, 0u, LV_PCT(5) );
  lv_obj_add_style( heading, &style, 0 );

  // Creating labels for individual slider current values
  red.slider_type = SLIDER_TYPE_RED;
  red.label = lv_label_create(act_scr);
  lv_label_set_text( red.label, "0");
  // lv_obj_align_to( red.label, slider_r, LV_ALIGN_TOP_MID, 0u, 0u );         // this will display inside slider
  lv_obj_align_to( red.label, slider_r, LV_ALIGN_OUT_TOP_MID, 0u, 0u );    // this will display outside slider

  green.slider_type = SLIDER_TYPE_GREEN;
  green.label = lv_label_create(act_scr);
  lv_label_set_text( green.label, "0");
  lv_obj_align_to( green.label, slider_g, LV_ALIGN_OUT_TOP_MID, 0u, 0u );

  blue.slider_type = SLIDER_TYPE_BLUE;
  blue.label = lv_label_create(act_scr);
  lv_label_set_text( blue.label, "0");
  lv_obj_align_to( blue.label, slider_b, LV_ALIGN_OUT_TOP_MID, 0u, 0u );

  // add event callbacks for sliders
  lv_obj_add_event_cb( slider_r, slider_callback, LV_EVENT_VALUE_CHANGED, &red );
  lv_obj_add_event_cb( slider_g, slider_callback, LV_EVENT_VALUE_CHANGED, &green );
  lv_obj_add_event_cb( slider_b, slider_callback, LV_EVENT_VALUE_CHANGED, &blue );
}

/*--------------------------Private Function Definitions----------------------*/
static void slider_callback( lv_event_t *e )
{
  static int32_t red, green, blue;
  int32_t slider_value = 0;
  // get the object (slider) for which the we received the event
  lv_obj_t *slider = lv_event_get_target(e);
  // extract the object (slider) user data
  RGB_Mixer_s *user_data = lv_event_get_user_data(e);
  // get the current slider value
  slider_value = lv_slider_get_value(slider);

  // now we have to update the slider labels
  lv_label_set_text_fmt( user_data->label, "%ld", slider_value );

  // now we have to update the color of the rectangle object based on the current
  // selected values of the color
  if( user_data->slider_type == SLIDER_TYPE_RED )
  {
    red = slider_value;
  }
  else if( user_data->slider_type == SLIDER_TYPE_GREEN )
  {
    green = slider_value;
  }
  else if( user_data->slider_type == SLIDER_TYPE_BLUE )
  {
    blue = slider_value;
  }
  // now we have the color information, update the rectangle color
  // NOTE: rectangle object must be file global else we will not be able to
  // update it from here
  lv_obj_set_style_bg_color( rectangle, lv_color_make( red, green, blue), LV_PART_MAIN);
  // the function `lv_color_make` can form colors if red, green and blue color
  // are specified
}

