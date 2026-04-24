/*
 * display.h ? ST7796S 480�320 LCD driver interface
 * ESE3500 Final Project ? Guitar Synthesizer Controller
 * Team 3: Synth Specialist (Guitar Hero Edition)
 * University of Pennsylvania ? Spring 2026
 *
 * Authors: Adam Shalabi, Brandon Parkansky, Panos Dimtsoudis
*/

#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdint.h>

void display_init(void);

void display_force_redraw(void);

void display_move_button_selection(int8_t dir);

void display_move_note_selection(int8_t dir, uint32_t now_ms);

void display_commit_selected_note(void);

uint8_t display_get_button_note(uint8_t btn);

uint8_t display_get_selected_button(void);

uint8_t display_get_selected_note(void);

#endif