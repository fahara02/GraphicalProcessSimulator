// Created by http://oleddisplay.squix.ch/ Consider a donation
// In case of problems make sure that you are using the font file with the
// correct version!
#pragma once
#include "TFT_eSPI.h"
#include "cstdint"
#include <pgmspace.h>
const uint8_t Orbitron_Medium_18Bitmaps[] PROGMEM = {

    // Bitmap Data:
    0x00,                         // ' '
    0xDB, 0x6D, 0xB6, 0xC0, 0x6C, // '!'
    0xDB, 0x6D, 0x80,             // '"'
    0x0C, 0x30, 0x30, 0xC0, 0xC6, 0x3F, 0xFE, 0xFF, 0xF8, 0x63, 0x03,
    0x0C, 0x0C, 0x30, 0xFF, 0xF3, 0xFF, 0xC6, 0x18, 0x18, 0x60, 0xC3,
    0x00, // '#'
    0x06, 0x00, 0x18, 0x07, 0xFF, 0x3F, 0xFC, 0xC6, 0x1B, 0x18, 0x0C,
    0x60, 0x31, 0x80, 0xFF, 0xF0, 0x18, 0xE0, 0x61, 0x81, 0x86, 0xC6,
    0x1B, 0xFF, 0xC7, 0xFF, 0x01, 0x80, 0x06, 0x00, // '$'
    0x78, 0x04, 0xFC, 0x0C, 0x84, 0x1C, 0x84, 0x38, 0xCC, 0x70, 0x78,
    0xE0, 0x03, 0x80, 0x07, 0x3E, 0x0E, 0x62, 0x1C, 0x62, 0x38, 0x62,
    0x60, 0x7E, 0x40, 0x3E, // '%'
    0x3F, 0xE0, 0x7F, 0xF0, 0x60, 0x30, 0x60, 0x00, 0x60, 0x00, 0x70,
    0x00, 0xFC, 0x00, 0xCF, 0x18, 0xC3, 0xD8, 0xC0, 0xF8, 0xC0, 0x3C,
    0xFF, 0xFE, 0x7F, 0xF2,                               // '&'
    0xDB, 0x00,                                           // '''
    0x6E, 0xCC, 0xCC, 0xCC, 0xCC, 0xCE, 0x60,             // '('
    0xCE, 0x66, 0x66, 0x66, 0x66, 0x6E, 0xC0,             // ')'
    0x18, 0x0C, 0x1E, 0xDF, 0xE1, 0xC1, 0xF1, 0xD8, 0x48, // '*'
    0x18, 0x0C, 0x06, 0x1F, 0xEF, 0xF0, 0xC0, 0x60,       // '+'
    0xDB, 0x40,                                           // ','
    0xFE, 0xFE,                                           // '-'
    0xD8,                                                 // '.'
    0x00, 0x80, 0x20, 0x18, 0x0C, 0x06, 0x03, 0x80, 0xC0, 0x60, 0x30,
    0x18, 0x0C, 0x03, 0x00, 0x80, 0x00, // '/'
    0x7F, 0xF3, 0xFF, 0xEC, 0x03, 0xB0, 0x1E, 0xC1, 0xDB, 0x0E, 0x6C,
    0x71, 0xB3, 0x86, 0xDC, 0x1B, 0xC0, 0x6E, 0x01, 0xBF, 0xFE, 0x7F,
    0xF0, // '0'
    0x1C, 0x78, 0xF3, 0x68, 0xC1, 0x83, 0x06, 0x0C, 0x18, 0x30, 0x60,
    0xC0, // '1'
    0x7F, 0xF3, 0xFF, 0xEC, 0x01, 0x80, 0x06, 0x00, 0x18, 0x00, 0x67,
    0xFF, 0xBF, 0xFC, 0xC0, 0x03, 0x00, 0x0C, 0x00, 0x3F, 0xFE, 0xFF,
    0xF8, // '2'
    0x7F, 0xE3, 0xFF, 0xCC, 0x03, 0x00, 0x0C, 0x00, 0x30, 0xFF, 0xC3,
    0xFF, 0x80, 0x06, 0x00, 0x18, 0x00, 0x6C, 0x01, 0xBF, 0xFE, 0x7F,
    0xF0, // '3'
    0x00, 0xE0, 0x07, 0x80, 0x3E, 0x01, 0xD8, 0x0E, 0x60, 0x71, 0x83,
    0x06, 0x38, 0x18, 0xFF, 0xFB, 0xFF, 0xE0, 0x06, 0x00, 0x18, 0x00,
    0x60, // '4'
    0xFF, 0xFB, 0xFF, 0xEC, 0x00, 0x30, 0x00, 0xC0, 0x03, 0xFF, 0x8F,
    0xFF, 0x80, 0x06, 0x00, 0x18, 0x00, 0x6C, 0x01, 0xBF, 0xFE, 0x7F,
    0xF0, // '5'
    0x7F, 0xE3, 0xFF, 0x8C, 0x00, 0x30, 0x00, 0xC0, 0x03, 0xFF, 0x8F,
    0xFF, 0xB0, 0x06, 0xC0, 0x1B, 0x00, 0x6C, 0x01, 0xBF, 0xFE, 0x7F,
    0xF0, // '6'
    0xFF, 0xCF, 0xFE, 0x00, 0x60, 0x06, 0x00, 0x60, 0x06, 0x00, 0x60,
    0x06, 0x00, 0x60, 0x06, 0x00, 0x60, 0x06, 0x00, 0x60, // '7'
    0x7F, 0xF3, 0xFF, 0xEC, 0x01, 0xB0, 0x06, 0xC0, 0x1B, 0x00, 0x6F,
    0xFF, 0xB0, 0x06, 0xC0, 0x1B, 0x00, 0x6C, 0x01, 0xBF, 0xFE, 0x7F,
    0xF0, // '8'
    0x7F, 0xF3, 0xFF, 0xEC, 0x01, 0xB0, 0x06, 0xC0, 0x1B, 0x00, 0x6F,
    0xFF, 0x9F, 0xFE, 0x00, 0x18, 0x00, 0x60, 0x01, 0xBF, 0xFE, 0x7F,
    0xF0,                                                             // '9'
    0xD8, 0x00, 0x00, 0x1B, 0x00,                                     // ':'
    0xD8, 0x00, 0x00, 0x1B, 0x68,                                     // ';'
    0x02, 0x02, 0x0E, 0x3C, 0xF0, 0xE0, 0xF0, 0x38, 0x1E, 0x06, 0x02, // '<'
    0xFF, 0xBF, 0xE0, 0x03, 0xFE, 0xFF, 0x80,                         // '='
    0x80, 0xC0, 0xF0, 0x78, 0x1E, 0x06, 0x0E, 0x3C, 0xF0, 0xE0, 0x80, // '>'
    0xFF, 0xCF, 0xFE, 0x00, 0x60, 0x06, 0x00, 0x60, 0x06, 0x1F, 0xE3,
    0xFC, 0x60, 0x00, 0x00, 0x00, 0x06, 0x00, 0x60, 0x00, // '?'
    0x7F, 0xF3, 0xFF, 0xEC, 0x01, 0xB1, 0xC6, 0xCF, 0x9B, 0x63, 0x6D,
    0x8D, 0xB6, 0x36, 0xCF, 0xFB, 0x1F, 0xEC, 0x00, 0x3F, 0xFE, 0x7F,
    0xF8, // '@'
    0x7F, 0xF3, 0xFF, 0xEC, 0x01, 0xB0, 0x06, 0xC0, 0x1B, 0x00, 0x6C,
    0x01, 0xBF, 0xFE, 0xC0, 0x1B, 0x00, 0x6C, 0x01, 0xB0, 0x06, 0xC0,
    0x18, // 'A'
    0xFF, 0xF3, 0xFF, 0xCC, 0x01, 0x30, 0x04, 0xC0, 0x13, 0xFF, 0xCF,
    0xFF, 0xB0, 0x06, 0xC0, 0x1B, 0x00, 0x6C, 0x01, 0xBF, 0xFE, 0xFF,
    0xF0, // 'B'
    0x7F, 0xFB, 0xFF, 0xEC, 0x00, 0x30, 0x00, 0xC0, 0x03, 0x00, 0x0C,
    0x00, 0x30, 0x00, 0xC0, 0x03, 0x00, 0x0C, 0x00, 0x3F, 0xFE, 0x7F,
    0xF8, // 'C'
    0xFF, 0xF3, 0xFF, 0xEC, 0x01, 0xB0, 0x06, 0xC0, 0x1B, 0x00, 0x6C,
    0x01, 0xB0, 0x06, 0xC0, 0x1B, 0x00, 0x6C, 0x01, 0xBF, 0xFE, 0xFF,
    0xF0, // 'D'
    0xFF, 0xF7, 0xFF, 0xB0, 0x01, 0x80, 0x0C, 0x00, 0x60, 0x03, 0xFF,
    0x18, 0x00, 0xC0, 0x06, 0x00, 0x30, 0x01, 0xFF, 0xEF, 0xFF, 0x00, // 'E'
    0xFF, 0xF7, 0xFF, 0xB0, 0x01, 0x80, 0x0C, 0x00, 0x60, 0x03, 0xFF,
    0x18, 0x00, 0xC0, 0x06, 0x00, 0x30, 0x01, 0x80, 0x0C, 0x00, 0x00, // 'F'
    0x7F, 0xF3, 0xFF, 0xEC, 0x01, 0xB0, 0x00, 0xC0, 0x03, 0x00, 0x0C,
    0x0F, 0xB0, 0x3E, 0xC0, 0x1B, 0x00, 0x6C, 0x01, 0xBF, 0xFE, 0x7F,
    0xF0, // 'G'
    0xC0, 0x1B, 0x00, 0x6C, 0x01, 0xB0, 0x06, 0xC0, 0x1B, 0x00, 0x6F,
    0xFF, 0xB0, 0x06, 0xC0, 0x1B, 0x00, 0x6C, 0x01, 0xB0, 0x06, 0xC0,
    0x18,                         // 'H'
    0xDB, 0x6D, 0xB6, 0xDB, 0x6C, // 'I'
    0x00, 0x18, 0x00, 0x60, 0x01, 0x80, 0x06, 0x00, 0x18, 0x00, 0x60,
    0x01, 0x80, 0x06, 0x00, 0x1B, 0x00, 0x6C, 0x01, 0xBF, 0xFE, 0x7F,
    0xF0, // 'J'
    0xC0, 0x3B, 0x01, 0xCC, 0x0E, 0x30, 0x70, 0xC3, 0x83, 0x0C, 0x0F,
    0xE0, 0x30, 0xC0, 0xC3, 0x83, 0x07, 0x0C, 0x0E, 0x30, 0x18, 0xC0,
    0x38, // 'K'
    0xC0, 0x03, 0x00, 0x0C, 0x00, 0x30, 0x00, 0xC0, 0x03, 0x00, 0x0C,
    0x00, 0x30, 0x00, 0xC0, 0x03, 0x00, 0x0C, 0x00, 0x3F, 0xFE, 0xFF,
    0xF8, // 'L'
    0xE0, 0x0E, 0xF0, 0x1E, 0xF0, 0x3E, 0xD8, 0x76, 0xCC, 0x66, 0xCE,
    0xC6, 0xC7, 0xC6, 0xC3, 0x86, 0xC1, 0x06, 0xC0, 0x06, 0xC0, 0x06,
    0xC0, 0x06, 0xC0, 0x06, // 'M'
    0xE0, 0x1B, 0xC0, 0x6F, 0x01, 0xB6, 0x06, 0xCC, 0x1B, 0x38, 0x6C,
    0x71, 0xB0, 0xE6, 0xC1, 0x9B, 0x03, 0x6C, 0x07, 0xB0, 0x1E, 0xC0,
    0x38, // 'N'
    0x7F, 0xF3, 0xFF, 0xEC, 0x01, 0xB0, 0x06, 0xC0, 0x1B, 0x00, 0x6C,
    0x01, 0xB0, 0x06, 0xC0, 0x1B, 0x00, 0x6C, 0x01, 0xBF, 0xFE, 0x7F,
    0xF0, // 'O'
    0xFF, 0xF3, 0xFF, 0xEC, 0x01, 0xB0, 0x06, 0xC0, 0x1B, 0x00, 0x6F,
    0xFF, 0xBF, 0xFC, 0xC0, 0x03, 0x00, 0x0C, 0x00, 0x30, 0x00, 0xC0,
    0x00, // 'P'
    0x7F, 0xF0, 0xFF, 0xF8, 0xC0, 0x18, 0xC0, 0x18, 0xC0, 0x18, 0xC0,
    0x18, 0xC0, 0x18, 0xC0, 0x18, 0xC0, 0x18, 0xC0, 0x18, 0xC0, 0x18,
    0xFF, 0xFE, 0x7F, 0xFE, // 'Q'
    0xFF, 0xF3, 0xFF, 0xEC, 0x01, 0xB0, 0x06, 0xC0, 0x1B, 0x00, 0x6F,
    0xFF, 0xBF, 0xFC, 0xC1, 0x83, 0x03, 0x0C, 0x06, 0x30, 0x1C, 0xC0,
    0x38, // 'R'
    0x7F, 0xF3, 0xFF, 0xEC, 0x01, 0xB0, 0x00, 0xC0, 0x03, 0x00, 0x0F,
    0xFF, 0x00, 0x06, 0x00, 0x18, 0x00, 0x6C, 0x01, 0xBF, 0xFE, 0x7F,
    0xF0, // 'S'
    0xFF, 0xFB, 0xFF, 0xE0, 0x30, 0x00, 0xC0, 0x03, 0x00, 0x0C, 0x00,
    0x30, 0x00, 0xC0, 0x03, 0x00, 0x0C, 0x00, 0x30, 0x00, 0xC0, 0x03,
    0x00, // 'T'
    0xC0, 0x1B, 0x00, 0x6C, 0x01, 0xB0, 0x06, 0xC0, 0x1B, 0x00, 0x6C,
    0x01, 0xB0, 0x06, 0xC0, 0x1B, 0x00, 0x6C, 0x01, 0xBF, 0xFE, 0x7F,
    0xF0, // 'U'
    0xC0, 0x03, 0xB8, 0x01, 0xC6, 0x00, 0x60, 0xC0, 0x38, 0x30, 0x0C,
    0x06, 0x06, 0x01, 0xC1, 0x80, 0x30, 0xC0, 0x06, 0x70, 0x01, 0x98,
    0x00, 0x3E, 0x00, 0x0F, 0x00, 0x01, 0x80, 0x00, // 'V'
    0xC0, 0xE0, 0x76, 0x07, 0x03, 0x18, 0x3C, 0x18, 0xC3, 0x61, 0x86,
    0x1B, 0x0C, 0x18, 0xCC, 0x60, 0xCC, 0x66, 0x06, 0x63, 0x30, 0x1F,
    0x0F, 0x80, 0xF0, 0x78, 0x07, 0x83, 0xC0, 0x18, 0x0C, 0x00, 0xC0,
    0x60, 0x00, // 'W'
    0xE0, 0x39, 0x81, 0xC3, 0x0E, 0x06, 0x30, 0x1D, 0x80, 0x3C, 0x00,
    0x70, 0x03, 0xC0, 0x1D, 0x80, 0x63, 0x03, 0x0E, 0x18, 0x1C, 0xE0,
    0x38, // 'X'
    0xE0, 0x0C, 0xE0, 0x30, 0xE0, 0xC0, 0xC3, 0x80, 0xCE, 0x01, 0xD8,
    0x01, 0xE0, 0x01, 0xC0, 0x03, 0x00, 0x06, 0x00, 0x0C, 0x00, 0x18,
    0x00, 0x30, 0x00, // 'Y'
    0xFF, 0xFB, 0xFF, 0xE0, 0x03, 0x80, 0x3C, 0x01, 0xC0, 0x0E, 0x00,
    0x70, 0x03, 0x80, 0x1C, 0x01, 0xC0, 0x0E, 0x00, 0x3F, 0xFE, 0xFF,
    0xF8,                                     // 'Z'
    0xEE, 0xCC, 0xCC, 0xCC, 0xCC, 0xCE, 0xE0, // '['
    0x80, 0x30, 0x0C, 0x01, 0x80, 0x30, 0x06, 0x00, 0xC0, 0x38, 0x06,
    0x00, 0xC0, 0x18, 0x02, 0x00, 0x80,       // '\'
    0xEE, 0x66, 0x66, 0x66, 0x66, 0x6E, 0xE0, // ']'
    0x00,                                     // '^'
    0xFF, 0xFB, 0xFF, 0xE0,                   // '_'
    0xDB, 0x00,                               // '`'
    0xFF, 0xCF, 0xFE, 0x00, 0x60, 0x06, 0xFF, 0xEF, 0xFE, 0xC0, 0x6C,
    0x06, 0xFF, 0xE7, 0xFE, // 'a'
    0xC0, 0x0C, 0x00, 0xC0, 0x0C, 0x00, 0xFF, 0xCC, 0x0E, 0xC0, 0x6C,
    0x06, 0xC0, 0x6C, 0x06, 0xC0, 0x6C, 0x06, 0xFF, 0xCF, 0xFC, // 'b'
    0xFF, 0xEC, 0x00, 0xC0, 0x0C, 0x00, 0xC0, 0x0C, 0x00, 0xC0, 0x0C,
    0x00, 0xFF, 0xE7, 0xFE, // 'c'
    0x00, 0x60, 0x06, 0x00, 0x60, 0x06, 0x7F, 0xEE, 0x06, 0xC0, 0x6C,
    0x06, 0xC0, 0x6C, 0x06, 0xC0, 0x6C, 0x06, 0x7F, 0xE7, 0xFE, // 'd'
    0xFF, 0xCC, 0x0C, 0xC0, 0x6C, 0x06, 0xFF, 0xEF, 0xFE, 0xC0, 0x0C,
    0x00, 0xFF, 0xE7, 0xFE, // 'e'
    0x7D, 0xFB, 0x06, 0x0F, 0xD8, 0x30, 0x60, 0xC1, 0x83, 0x06, 0x0C,
    0x18, 0x00, // 'f'
    0xFF, 0xD8, 0x1B, 0x03, 0x60, 0x6C, 0x0D, 0x81, 0xB0, 0x36, 0x06,
    0xFF, 0xCF, 0xF8, 0x03, 0x00, 0x63, 0xFC, 0x7F, 0x80, // 'g'
    0xC0, 0x0C, 0x00, 0xC0, 0x0C, 0x00, 0xFF, 0xCC, 0x0E, 0xC0, 0x6C,
    0x06, 0xC0, 0x6C, 0x06, 0xC0, 0x6C, 0x06, 0xC0, 0x6C, 0x06, // 'h'
    0xD8, 0x0D, 0xB6, 0xDB, 0x6D, 0x80,                         // 'i'
    0x0C, 0x18, 0x00, 0x00, 0xC1, 0x83, 0x06, 0x0C, 0x18, 0x30, 0x60,
    0xC1, 0x83, 0x06, 0xFD, 0xF8, // 'j'
    0xC0, 0x18, 0x03, 0x00, 0x60, 0x0C, 0x1D, 0x87, 0x31, 0xC6, 0x30,
    0xFC, 0x1F, 0xC3, 0x1C, 0x61, 0x8C, 0x19, 0x81, 0x80, // 'k'
    0xC6, 0x31, 0x8C, 0x63, 0x18, 0xC6, 0x31, 0x8F, 0x38, // 'l'
    0xFF, 0xFE, 0x61, 0xC3, 0xB0, 0x60, 0xD8, 0x30, 0x6C, 0x18, 0x36,
    0x0C, 0x1B, 0x06, 0x0D, 0x83, 0x06, 0xC1, 0x83, 0x60, 0xC1, 0x80, // 'm'
    0xFF, 0xCC, 0x0E, 0xC0, 0x6C, 0x06, 0xC0, 0x6C, 0x06, 0xC0, 0x6C,
    0x06, 0xC0, 0x6C, 0x06, // 'n'
    0xFF, 0xCC, 0x0C, 0xC0, 0x6C, 0x06, 0xC0, 0x6C, 0x06, 0xC0, 0x6C,
    0x06, 0xFF, 0xC7, 0xFC, // 'o'
    0xFF, 0xCC, 0x0E, 0xC0, 0x6C, 0x06, 0xC0, 0x6C, 0x06, 0xC0, 0x6C,
    0x06, 0xFF, 0xCF, 0xFC, 0xC0, 0x0C, 0x00, 0xC0, 0x0C, 0x00, // 'p'
    0x7F, 0xEE, 0x06, 0xC0, 0x6C, 0x06, 0xC0, 0x6C, 0x06, 0xC0, 0x6C,
    0x06, 0x7F, 0xE7, 0xFE, 0x00, 0x60, 0x06, 0x00, 0x60, 0x06, // 'q'
    0xFF, 0x60, 0x30, 0x18, 0x0C, 0x06, 0x03, 0x01, 0x80, 0xC0, 0x60,
    0x00, // 'r'
    0xFF, 0xD8, 0x1B, 0x00, 0x60, 0x0F, 0xFC, 0xFF, 0x80, 0x16, 0x02,
    0xFF, 0xCF, 0xF8, // 's'
    0xC1, 0x83, 0x06, 0x0F, 0xD8, 0x30, 0x60, 0xC1, 0x83, 0x06, 0x0F,
    0xCF, 0x80, // 't'
    0xC0, 0x6C, 0x06, 0xC0, 0x6C, 0x06, 0xC0, 0x6C, 0x06, 0xC0, 0x6C,
    0x06, 0xFF, 0xC7, 0xFC, // 'u'
    0xE0, 0x1C, 0xE0, 0x30, 0xC0, 0xC0, 0xC3, 0x81, 0x86, 0x01, 0x9C,
    0x03, 0x30, 0x03, 0xC0, 0x07, 0x80, 0x06, 0x00, // 'v'
    0xC1, 0xC1, 0xD8, 0x3C, 0x31, 0x8F, 0x8E, 0x31, 0xB9, 0x83, 0x73,
    0x30, 0x6C, 0x6C, 0x0F, 0x87, 0x80, 0xE0, 0xF0, 0x1C, 0x1C, 0x01,
    0x81, 0x80, // 'w'
    0xE0, 0xE7, 0x18, 0x3B, 0x01, 0xE0, 0x0E, 0x01, 0xE0, 0x3F, 0x03,
    0x38, 0x61, 0xCE, 0x0E, // 'x'
    0xC0, 0xD8, 0x1B, 0x03, 0x60, 0x6C, 0x0D, 0x81, 0xB0, 0x36, 0x06,
    0xFF, 0xCF, 0xF8, 0x03, 0x00, 0x63, 0xFC, 0x7F, 0x80, // 'y'
    0xFF, 0xEF, 0xFE, 0x01, 0xC0, 0x70, 0x0E, 0x01, 0xC0, 0x38, 0x0E,
    0x00, 0xFF, 0xEF, 0xFE,                               // 'z'
    0x33, 0x98, 0xC6, 0x73, 0x0C, 0x63, 0x18, 0xE3, 0x00, // '{'
    0xDB, 0x6D, 0xB6, 0xDB, 0x6D, 0xB6, 0xC0,             // '|'
    0xC7, 0x18, 0xC6, 0x38, 0xCE, 0x63, 0x19, 0xCC, 0x00  // '}'
};
const GFXglyph Orbitron_Medium_18Glyphs[] PROGMEM = {
    // bitmapOffset, width, height, xAdvance, xOffset, yOffset
    {0, 1, 1, 6, 0, 0},         // ' '
    {1, 3, 13, 5, 1, -13},      // '!'
    {6, 6, 3, 8, 1, -13},       // '"'
    {9, 14, 13, 15, 1, -13},    // '#'
    {32, 14, 17, 15, 1, -15},   // '$'
    {62, 16, 13, 18, 1, -13},   // '%'
    {88, 16, 13, 18, 1, -13},   // '&'
    {114, 3, 3, 5, 1, -13},     // '''
    {116, 4, 13, 6, 1, -13},    // '('
    {123, 4, 13, 6, 1, -13},    // ')'
    {130, 9, 8, 10, 0, -13},    // '*'
    {139, 9, 7, 9, 0, -9},      // '+'
    {147, 3, 4, 4, 1, -2},      // ','
    {149, 8, 2, 10, 1, -6},     // '-'
    {151, 3, 2, 5, 1, -2},      // '.'
    {152, 10, 13, 10, 0, -13},  // '/'
    {169, 14, 13, 16, 1, -13},  // '0'
    {192, 7, 13, 8, 0, -13},    // '1'
    {204, 14, 13, 16, 1, -13},  // '2'
    {227, 14, 13, 16, 1, -13},  // '3'
    {250, 14, 13, 14, 0, -13},  // '4'
    {273, 14, 13, 16, 1, -13},  // '5'
    {296, 14, 13, 16, 1, -13},  // '6'
    {319, 12, 13, 13, 0, -13},  // '7'
    {339, 14, 13, 16, 1, -13},  // '8'
    {362, 14, 13, 16, 1, -13},  // '9'
    {385, 3, 11, 5, 1, -11},    // ':'
    {390, 3, 13, 4, 1, -11},    // ';'
    {395, 8, 11, 10, 0, -11},   // '<'
    {406, 10, 5, 12, 1, -8},    // '='
    {413, 8, 11, 10, 1, -11},   // '>'
    {424, 12, 13, 13, 1, -13},  // '?'
    {444, 14, 13, 16, 1, -13},  // '@'
    {467, 14, 13, 16, 1, -13},  // 'A'
    {490, 14, 13, 16, 1, -13},  // 'B'
    {513, 14, 13, 16, 1, -13},  // 'C'
    {536, 14, 13, 16, 1, -13},  // 'D'
    {559, 13, 13, 15, 1, -13},  // 'E'
    {581, 13, 13, 14, 1, -13},  // 'F'
    {603, 14, 13, 16, 1, -13},  // 'G'
    {626, 14, 13, 16, 1, -13},  // 'H'
    {649, 3, 13, 5, 1, -13},    // 'I'
    {654, 14, 13, 15, 0, -13},  // 'J'
    {677, 14, 13, 15, 1, -13},  // 'K'
    {700, 14, 13, 15, 1, -13},  // 'L'
    {723, 16, 13, 18, 1, -13},  // 'M'
    {749, 14, 13, 16, 1, -13},  // 'N'
    {772, 14, 13, 16, 1, -13},  // 'O'
    {795, 14, 13, 15, 1, -13},  // 'P'
    {818, 16, 13, 17, 1, -13},  // 'Q'
    {844, 14, 13, 16, 1, -13},  // 'R'
    {867, 14, 13, 16, 1, -13},  // 'S'
    {890, 14, 13, 15, 0, -13},  // 'T'
    {913, 14, 13, 16, 1, -13},  // 'U'
    {936, 18, 13, 19, 1, -13},  // 'V'
    {966, 21, 13, 22, 1, -13},  // 'W'
    {1001, 14, 13, 16, 1, -13}, // 'X'
    {1024, 15, 13, 16, 0, -13}, // 'Y'
    {1049, 14, 13, 16, 1, -13}, // 'Z'
    {1072, 4, 13, 6, 1, -13},   // '['
    {1079, 10, 13, 10, 0, -13}, // '\'
    {1096, 4, 13, 6, 1, -13},   // ']'
    {1103, 1, 1, 1, 0, 0},      // '^'
    {1104, 14, 2, 16, 1, 0},    // '_'
    {1108, 3, 3, 5, 1, -18},    // '`'
    {1110, 12, 10, 13, 1, -10}, // 'a'
    {1125, 12, 14, 13, 1, -14}, // 'b'
    {1146, 12, 10, 14, 1, -10}, // 'c'
    {1161, 12, 14, 13, 0, -14}, // 'd'
    {1182, 12, 10, 13, 1, -10}, // 'e'
    {1197, 7, 14, 8, 1, -14},   // 'f'
    {1210, 11, 14, 13, 1, -10}, // 'g'
    {1230, 12, 14, 13, 1, -14}, // 'h'
    {1251, 3, 14, 5, 1, -14},   // 'i'
    {1257, 7, 18, 5, -3, -14},  // 'j'
    {1273, 11, 14, 13, 1, -14}, // 'k'
    {1293, 5, 14, 6, 1, -14},   // 'l'
    {1302, 17, 10, 19, 1, -10}, // 'm'
    {1324, 12, 10, 14, 1, -10}, // 'n'
    {1339, 12, 10, 13, 1, -10}, // 'o'
    {1354, 12, 14, 13, 1, -10}, // 'p'
    {1375, 12, 14, 13, 0, -10}, // 'q'
    {1396, 9, 10, 10, 1, -10},  // 'r'
    {1408, 11, 10, 13, 1, -10}, // 's'
    {1422, 7, 14, 8, 1, -14},   // 't'
    {1435, 12, 10, 14, 1, -10}, // 'u'
    {1450, 15, 10, 15, 0, -10}, // 'v'
    {1469, 19, 10, 20, 1, -10}, // 'w'
    {1493, 12, 10, 13, 1, -10}, // 'x'
    {1508, 11, 14, 13, 1, -10}, // 'y'
    {1528, 12, 10, 14, 1, -10}, // 'z'
    {1543, 5, 13, 6, 0, -13},   // '{'
    {1552, 3, 17, 5, 1, -15},   // '|'
    {1559, 5, 13, 6, 1, -13}    // '}'
};
const GFXfont Orbitron_Medium_18 PROGMEM = {
    (uint8_t *)Orbitron_Medium_18Bitmaps, (GFXglyph *)Orbitron_Medium_18Glyphs,
    0x20, 0x7E, 19};