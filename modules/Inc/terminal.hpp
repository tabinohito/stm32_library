#pragma once

#include <stdio.h>

namespace terminal
{

/**
* @brief 文字・背景の色
*/
enum class TerminalColor
{
    Black = 0,
    Red,
    Green,
    Yellow,
    Blue,
    Magenta,
    Cyan,
    Gray = 7,
    Default = 9
};

/**
* @brief カーソルの移動方向
*
*/
enum class TerminalCursorDir
{
    Up,
    Down,
    Right,
    Left
};


/**
* @brief 文字の色を指定
*
* @param c
*/
static void set_letter_color(TerminalColor c) { printf("\033[%dm", 30 + (int)c); }

/**
* @brief 背景の色を指定
*
* @param c
*/
static void set_back_color(TerminalColor c)   { printf("\033[%dm", 40 + (int)c); }

/**
* @brief 文字の効果をなくす
*
*/
static void set_letter_default()       { printf("\033[0m"); }

/**
* @brief ボールド体効果のセット
*
* @param is_on true: ボールド体効果On
*/
static void set_bright_bold(bool is_on = true) { printf("\033[1m"); }

/**
* @brief 下線効果のセット
*
* @param is_on true: 下線効果On
*/
static void set_under_line(bool is_on = true)  { printf("\033[4m"); }

/**
* @brief 色反転の効果のセット
*
* @param is_on true: 色反転の効果On
*/
static void set_flip(bool is_on = true)       { printf("\033[7m"); }

/**
* @brief 画面の消去
*
*/
static void clear()                { printf("\033[2J"); }

/**
* @brief カーソル位置の指定
*
* @param x
* @param y
*/
static void set_pos(int x, int y)   { printf("\033[%d;%dH", y, x); }

/**
* @brief 現在のカーソル位置からその行の後ろを消去
*
*/
static void clear_to_line_end()       { printf("\033[K"); }

/**
* @brief 現在のカーソル位置からその行の後ろを消去してから改行
*
*/
static void endl()                 { clear_to_line_end(); printf("\n");}

/**
* @brief カーソルを上に移動
*
* @param distance 移動距離
*/
static void cursor_up   (int distance) { printf("\033[%dA", distance); }

/**
* @brief カーソルを下に移動
*
* @param distance 移動距離
*/
static void cursor_down (int distance) { printf("\033[%dB", distance); }

/**
* @brief カーソルを右に移動
*
* @param distance 移動距離
*/
static void cursor_right(int distance) { printf("\033[%dC", distance); }

/**
* @brief カーソルを左に移動
*
* @param distance 移動距離
*/
static void cursor_left (int distance) { printf("\033[%dD", distance); }

/**
* @brief カーソルを指定した分だけ移動
*
* @param dir 移動方向
* @param distance 移動距離
*/
static void move_cursor(TerminalCursorDir dir, int distance)
{
    switch(dir)
    {
        case TerminalCursorDir::Up:      cursor_up(distance);    break;
        case TerminalCursorDir::Down:    cursor_down(distance);  break;
        case TerminalCursorDir::Right:   cursor_right(distance); break;
        case TerminalCursorDir::Left:    cursor_left(distance);  break;
    }
}

/**
* @brief 画面の消去・カーソルの位置を原点に移動
*
*/
static void cls()
{
    clear();
    set_pos(0, 0);
}

/**
* @brief 画面の消去と色・効果の初期化
*
*/
static void reset()
{
    cls();
    set_letter_color(TerminalColor::Default);
    set_back_color(TerminalColor::Default);
    set_letter_default();
}

}