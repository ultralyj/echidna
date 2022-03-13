/**
 * @file tjrc_fontCode.c
 * @author YYYDS team (1951578@tongji.edu.cn)
 * @brief 6*12 ASCII字库文件
 * @version 0.1
 * @date 2022-03-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "stdint.h"

#define ASCII_NUM 96
#define ASCII_EACH_SIZE 12
/*
 * ASCII 6_12 字模库.去除了前32个控制字符。
 * 扫描方式：由左到右，由上到下。宽度按字节对齐。
 * 存储方式: 高位在前。
*/
const uint8_t ASCII_6_12[ASCII_NUM * ASCII_EACH_SIZE]={
    /* 0x20 [ ] */
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    /* 0x21 [!] */
        0x00,0x00,0x20,0x20,0x20,0x20,0x20,0x00,0x00,0x20,0x00,0x00,
    /* 0x22 ["] */
        0x28,0x28,0x50,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    /* 0x23 [#] */
        0x00,0x00,0x50,0x50,0xF8,0x50,0x50,0xF8,0x50,0x50,0x00,0x00,
    /* 0x24 [$] */
        0x00,0x20,0x70,0xA8,0xA0,0x60,0x30,0x28,0xA8,0x70,0x20,0x00,
    /* 0x25 [%] */
        0x00,0x00,0x48,0xA8,0xB0,0xA8,0x74,0x34,0x54,0x48,0x00,0x00,
    /* 0x26 [&] */
        0x00,0x00,0x20,0x50,0x50,0x6C,0xA8,0xA8,0x94,0x68,0x00,0x00,
    /* 0x27 ['] */
        0x40,0x40,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    /* 0x28 [(] */
        0x08,0x10,0x10,0x20,0x20,0x20,0x20,0x20,0x10,0x10,0x08,0x00,
    /* 0x29 [)] */
        0x40,0x20,0x20,0x10,0x10,0x10,0x10,0x10,0x20,0x20,0x40,0x00,
    /* 0x2A [*] */
        0x00,0x00,0x00,0x20,0xA8,0x70,0x70,0xA8,0x20,0x00,0x00,0x00,
    /* 0x2B [+] */
        0x00,0x00,0x00,0x10,0x10,0x7C,0x10,0x10,0x00,0x00,0x00,0x00,
    /* 0x2C [,] */
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x40,0x40,0x80,0x00,
    /* 0x2D [-] */
        0x00,0x00,0x00,0x00,0x00,0xFC,0x00,0x00,0x00,0x00,0x00,0x00,
    /* 0x2E [.] */
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x40,0x00,0x00,
    /* 0x2F [/] */
        0x00,0x04,0x08,0x08,0x10,0x10,0x20,0x20,0x40,0x40,0x80,0x00,
    /* 0x30 [0] */
        0x00,0x00,0x70,0x88,0x88,0x88,0x88,0x88,0x88,0x70,0x00,0x00,
    /* 0x31 [1] */
        0x00,0x00,0x20,0x60,0x20,0x20,0x20,0x20,0x20,0x70,0x00,0x00,
    /* 0x32 [2] */
        0x00,0x00,0x70,0x88,0x88,0x10,0x20,0x40,0x80,0xF8,0x00,0x00,
    /* 0x33 [3] */
        0x00,0x00,0x70,0x88,0x08,0x30,0x08,0x08,0x88,0x70,0x00,0x00,
    /* 0x34 [4] */
        0x00,0x00,0x10,0x30,0x30,0x50,0x90,0xF8,0x10,0x38,0x00,0x00,
    /* 0x35 [5] */
        0x00,0x00,0xF8,0x80,0x80,0xF0,0x88,0x08,0x88,0x70,0x00,0x00,
    /* 0x36 [6] */
        0x00,0x00,0x30,0x48,0x80,0xB0,0xC8,0x88,0x88,0x70,0x00,0x00,
    /* 0x37 [7] */
        0x00,0x00,0x78,0x08,0x10,0x10,0x20,0x20,0x20,0x20,0x00,0x00,
    /* 0x38 [8] */
        0x00,0x00,0x70,0x88,0x88,0x70,0x88,0x88,0x88,0x70,0x00,0x00,
    /* 0x39 [9] */
        0x00,0x00,0x70,0x88,0x88,0x98,0x68,0x08,0x90,0x60,0x00,0x00,
    /* 0x3A [:] */
        0x00,0x00,0x00,0x00,0x20,0x00,0x00,0x00,0x00,0x20,0x00,0x00,
    /* 0x3B [;] */
        0x00,0x00,0x00,0x00,0x00,0x20,0x00,0x00,0x00,0x20,0x20,0x00,
    /* 0x3C [<] */
        0x00,0x00,0x08,0x10,0x20,0x40,0x40,0x20,0x10,0x08,0x00,0x00,
    /* 0x3D [=] */
        0x00,0x00,0x00,0x00,0xFC,0x00,0xFC,0x00,0x00,0x00,0x00,0x00,
    /* 0x3E [>] */
        0x00,0x00,0x40,0x20,0x10,0x08,0x08,0x10,0x20,0x40,0x00,0x00,
    /* 0x3F [?] */
        0x00,0x00,0x70,0x88,0x88,0x10,0x20,0x20,0x00,0x20,0x00,0x00,
    /* 0x40 [@] */
        0x00,0x00,0x38,0x44,0x94,0xB4,0xB4,0xB8,0x44,0x38,0x00,0x00,
    /* 0x41 [A] */
        0x00,0x00,0x20,0x20,0x30,0x50,0x50,0x78,0x48,0xCC,0x00,0x00,
    /* 0x42 [B] */
        0x00,0x00,0xF0,0x48,0x48,0x70,0x48,0x48,0x48,0xF0,0x00,0x00,
    /* 0x43 [C] */
        0x00,0x00,0x78,0x88,0x80,0x80,0x80,0x80,0x88,0x70,0x00,0x00,
    /* 0x44 [D] */
        0x00,0x00,0xF0,0x48,0x48,0x48,0x48,0x48,0x48,0xF0,0x00,0x00,
    /* 0x45 [E] */
        0x00,0x00,0xF8,0x48,0x50,0x70,0x50,0x40,0x48,0xF8,0x00,0x00,
    /* 0x46 [F] */
        0x00,0x00,0xF8,0x48,0x50,0x70,0x50,0x40,0x40,0xE0,0x00,0x00,
    /* 0x47 [G] */
        0x00,0x00,0x38,0x48,0x80,0x80,0x9C,0x88,0x48,0x30,0x00,0x00,
    /* 0x48 [H] */
        0x00,0x00,0xCC,0x48,0x48,0x78,0x48,0x48,0x48,0xCC,0x00,0x00,
    /* 0x49 [I] */
        0x00,0x00,0xF8,0x20,0x20,0x20,0x20,0x20,0x20,0xF8,0x00,0x00,
    /* 0x4A [J] */
        0x00,0x00,0x7C,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x90,0xE0,
    /* 0x4B [K] */
        0x00,0x00,0xEC,0x48,0x50,0x60,0x50,0x48,0x48,0xEC,0x00,0x00,
    /* 0x4C [L] */
        0x00,0x00,0xE0,0x40,0x40,0x40,0x40,0x40,0x44,0xFC,0x00,0x00,
    /* 0x4D [M] */
        0x00,0x00,0xDC,0xD8,0xD8,0xD8,0xA8,0xA8,0xA8,0xAC,0x00,0x00,
    /* 0x4E [N] */
        0x00,0x00,0xDC,0x48,0x68,0x68,0x58,0x58,0x48,0xE8,0x00,0x00,
    /* 0x4F [O] */
        0x00,0x00,0x70,0x88,0x88,0x88,0x88,0x88,0x88,0x70,0x00,0x00,
    /* 0x50 [P] */
        0x00,0x00,0xF0,0x48,0x48,0x70,0x40,0x40,0x40,0xE0,0x00,0x00,
    /* 0x51 [Q] */
        0x00,0x00,0x70,0x88,0x88,0x88,0x88,0xE8,0x98,0x70,0x18,0x00,
    /* 0x52 [R] */
        0x00,0x00,0xF0,0x48,0x48,0x70,0x50,0x48,0x48,0xEC,0x00,0x00,
    /* 0x53 [S] */
        0x00,0x00,0x78,0x88,0x80,0x60,0x10,0x08,0x88,0xF0,0x00,0x00,
    /* 0x54 [T] */
        0x00,0x00,0xF8,0xA8,0x20,0x20,0x20,0x20,0x20,0x70,0x00,0x00,
    /* 0x55 [U] */
        0x00,0x00,0xCC,0x48,0x48,0x48,0x48,0x48,0x48,0x30,0x00,0x00,
    /* 0x56 [V] */
        0x00,0x00,0xCC,0x48,0x48,0x50,0x50,0x30,0x20,0x20,0x00,0x00,
    /* 0x57 [W] */
        0x00,0x00,0xA8,0xA8,0xA8,0xA8,0x70,0x50,0x50,0x50,0x00,0x00,
    /* 0x58 [X] */
        0x00,0x00,0xD8,0x50,0x50,0x20,0x20,0x50,0x50,0xD8,0x00,0x00,
    /* 0x59 [Y] */
        0x00,0x00,0xD8,0x50,0x50,0x50,0x20,0x20,0x20,0x70,0x00,0x00,
    /* 0x5A [Z] */
        0x00,0x00,0xF8,0x90,0x10,0x20,0x20,0x40,0x48,0xF8,0x00,0x00,
    /* 0x5B [[] */
        0x38,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x38,0x00,
    /* 0x5C [\] */
        0x00,0x40,0x40,0x20,0x20,0x20,0x10,0x10,0x10,0x08,0x08,0x00,
    /* 0x5D []] */
        0x70,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x70,0x00,
    /* 0x5E [^] */
        0x20,0x50,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    /* 0x5F [_] */
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFC,
    /* 0x60 [`] */
        0x40,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    /* 0x61 [a] */
        0x00,0x00,0x00,0x00,0x00,0x30,0x48,0x38,0x48,0x3C,0x00,0x00,
    /* 0x62 [b] */
        0x00,0xC0,0x40,0x40,0x40,0x70,0x48,0x48,0x48,0x70,0x00,0x00,
    /* 0x63 [c] */
        0x00,0x00,0x00,0x00,0x00,0x38,0x48,0x40,0x48,0x30,0x00,0x00,
    /* 0x64 [d] */
        0x00,0x18,0x08,0x08,0x08,0x38,0x48,0x48,0x48,0x3C,0x00,0x00,
    /* 0x65 [e] */
        0x00,0x00,0x00,0x00,0x00,0x30,0x48,0x78,0x40,0x38,0x00,0x00,
    /* 0x66 [f] */
        0x00,0x18,0x24,0x20,0x20,0x78,0x20,0x20,0x20,0x78,0x00,0x00,
    /* 0x67 [g] */
        0x00,0x00,0x00,0x00,0x00,0x3C,0x48,0x30,0x40,0x38,0x44,0x38,
    /* 0x68 [h] */
        0x00,0xC0,0x40,0x40,0x40,0x70,0x48,0x48,0x48,0xEC,0x00,0x00,
    /* 0x69 [i] */
        0x00,0x20,0x20,0x00,0x00,0x60,0x20,0x20,0x20,0x70,0x00,0x00,
    /* 0x6A [j] */
        0x00,0x10,0x10,0x00,0x00,0x30,0x10,0x10,0x10,0x10,0x10,0xE0,
    /* 0x6B [k] */
        0x00,0xC0,0x40,0x40,0x40,0x58,0x50,0x60,0x50,0xC8,0x00,0x00,
    /* 0x6C [l] */
        0x00,0xE0,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0xF8,0x00,0x00,
    /* 0x6D [m] */
        0x00,0x00,0x00,0x00,0x00,0xF0,0xA8,0xA8,0xA8,0xA8,0x00,0x00,
    /* 0x6E [n] */
        0x00,0x00,0x00,0x00,0x00,0xF0,0x48,0x48,0x48,0xEC,0x00,0x00,
    /* 0x6F [o] */
        0x00,0x00,0x00,0x00,0x00,0x30,0x48,0x48,0x48,0x30,0x00,0x00,
    /* 0x70 [p] */
        0x00,0x00,0x00,0x00,0x00,0xF0,0x48,0x48,0x48,0x70,0x40,0xE0,
    /* 0x71 [q] */
        0x00,0x00,0x00,0x00,0x00,0x38,0x48,0x48,0x48,0x38,0x08,0x1C,
    /* 0x72 [r] */
        0x00,0x00,0x00,0x00,0x00,0xD8,0x60,0x40,0x40,0xE0,0x00,0x00,
    /* 0x73 [s] */
        0x00,0x00,0x00,0x00,0x00,0x78,0x40,0x30,0x08,0x78,0x00,0x00,
    /* 0x74 [t] */
        0x00,0x00,0x00,0x20,0x20,0x78,0x20,0x20,0x20,0x38,0x00,0x00,
    /* 0x75 [u] */
        0x00,0x00,0x00,0x00,0x00,0xD8,0x48,0x48,0x48,0x3C,0x00,0x00,
    /* 0x76 [v] */
        0x00,0x00,0x00,0x00,0x00,0xD8,0x50,0x50,0x20,0x20,0x00,0x00,
    /* 0x77 [w] */
        0x00,0x00,0x00,0x00,0x00,0xA8,0xA8,0x70,0x50,0x50,0x00,0x00,
    /* 0x78 [x] */
        0x00,0x00,0x00,0x00,0x00,0xD8,0x50,0x20,0x50,0xD8,0x00,0x00,
    /* 0x79 [y] */
        0x00,0x00,0x00,0x00,0x00,0xCC,0x48,0x48,0x30,0x10,0x20,0xC0,
    /* 0x7A [z] */
        0x00,0x00,0x00,0x00,0x00,0x78,0x10,0x20,0x20,0x78,0x00,0x00,
    /* 0x7B [{] */
        0x18,0x10,0x10,0x10,0x10,0x30,0x10,0x10,0x10,0x10,0x18,0x00,
    /* 0x7C [|] */
        0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,
    /* 0x7D [}] */
        0x60,0x20,0x20,0x20,0x20,0x10,0x20,0x20,0x20,0x20,0x60,0x00,
    /* 0x7E [~] */
        0x68,0x90,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    /* 0x7F */
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    };



