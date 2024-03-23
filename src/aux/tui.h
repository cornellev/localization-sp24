// Copyright (C) 2024 Ethan Uppal. All rights reserved.
// Taken from ethanuppal.com
extern "C" {
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <poll.h>
namespace ethan_tui {
#define I STDIN_FILENO
#define O STDOUT_FILENO
#define P printf
#define U struct termios
#define E "\033"
#define Y(x) TC##x##FLUSH
    struct tui {
        int w;
        int h;
        char k;
        int f;
        U t;
        U u;
        char* b;
        struct pollfd p;
    };
#define tui_begin(T, W, H)                                                     \
    T.w = W;                                                                   \
    T.h = H;                                                                   \
    P(E "[?25l");                                                              \
    T.f = fcntl(I, F_GETFL);                                                   \
    tcgetattr(O, &T.t);                                                        \
    fcntl(I, F_SETFL, T.f | O_NONBLOCK);                                       \
    T.u = T.t;                                                                 \
    T.u.c_lflag &= ~(ECHO | ICANON);                                           \
    tcsetattr(O, Y(SA), &T.u);                                                 \
    T.p.fd = I;                                                                \
    T.p.events = POLLIN;                                                       \
    T.p.revents = 0;                                                           \
    T.b = (char*)malloc(W * H);                                                \
    memset(T.b, ' ', W* H);
#define tui_end(T)                                                             \
    P(E "[?25h");                                                              \
    fcntl(I, F_SETFL, T.f);                                                    \
    tcsetattr(O, Y(SA), &T.t);                                                 \
    free(T.b)
#define tui_keys(T)                                                            \
    T.k = 0;                                                                   \
    if (poll(&T.p, 1, 15) > 0) read(I, &T.k, 1)
#define tui_flush() tcflush(O, Y(IO))
#define tui_draw(T)                                                            \
    P(E "[H");                                                                 \
    for (int i = 0; i < T.h; i++) {                                            \
        for (int j = 0; j < T.w; j++) putchar(T.b[i * T.w + j]);               \
        putchar('\n');                                                         \
    }                                                                          \
    tui_flush()
}
}
