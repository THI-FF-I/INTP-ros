//
// Created by Johan on 14.11.2022.
//

#include "jps_maze_curses/jps_maze_curses.hpp"

namespace jps_maze_curses
{
    CursesWindow::CursesWindow(const char *port, const bool is_server, const bool team_A) : port(port), is_server(is_server), team_A(team_A), cur_player_index(PLAYER), row_buf(nullptr), cur_row(0)
    {
        if(this->is_server) {
            printf("Starting as visualizer for a server\n");
        } else if (this->team_A) {
            printf("Starting as visualizer for a team A member\n");
        } else {
            printf("Starting as visualizer for a team B member\n");
        }
        this->setup_socket();

        this->get_dim();

        printf("Got dimensions: width: %u, height: %u\n", this->width, this->height);

        printf("Allocating cur_row buffer of size %zu Bytes\n", sizeof(block_t) * this->width);
        this->row_buf = new block_t[this->width];

        this->init_curses_app();

        for (cur_row = 0; cur_row < this->height; ++cur_row)
        {
            this->get_row();
            this->print_row();
        }
        refresh();
    }

    void CursesWindow::setup_socket()
    {
        printf("Set up socket on port %s\n", this->port);

        struct addrinfo hints = {};
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_DGRAM;
        hints.ai_protocol = 0;
        hints.ai_flags = AI_PASSIVE;
        struct addrinfo *result, *rp;
        if (getaddrinfo(NULL, port, &hints, &result) != 0)
        {
            throw std::runtime_error("Could not resolve hostname error");
        }

        for (rp = result; rp != NULL; rp = rp->ai_next)
        {
            this->network_socket = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
            if (this->network_socket == -1)
            {
                continue;
            }
            printf("Successfully create a socket\n");

            if (bind(this->network_socket, rp->ai_addr, rp->ai_addrlen) == 0)
            {
                printf("Successfully bound socket to port %s\n", this->port);
                break;
            }
            printf("Failed to bind socket, closing\n");
            close(this->network_socket);
        }

        freeaddrinfo(result);

        if (rp == NULL)
        {
            throw std::runtime_error("Could not connect to visualisation server");
        }
    }

    CursesWindow::~CursesWindow()
    {
        standend();  // Turn off all attributes
        refresh();   // Write changes to terminal
        curs_set(1); // Set cursor state to normal visibility
        endwin();    // Terminate curses application
        printf("Deinitialized curses\n");
        printf("Closing socket\n");
        close(this->network_socket);
        printf("Freeing cur_row buffer\n");
        delete this->row_buf;
    }

    void CursesWindow::get_dim()
    {
        block_t dim[2];
        ssize_t size = recv(this->network_socket, dim, sizeof(dim), MSG_WAITALL);
        if (size == -1)
        {
            throw std::runtime_error("Failed receiving dim");
        }
        else if (size != sizeof(dim))
        {
            throw std::runtime_error("Received the wrong number of bytes for a row");
        }
        this->width = dim[0];
        this->height = dim[1];
    }

    void CursesWindow::check_dim()
    {
        block_t last_width = this->width;
        block_t last_height = this->height;
        this->get_dim();
        if (last_width != this->width && last_height == this->height)
        {
            this->print_result(true);
            while(true);
        }
        else if (last_width == this->width && last_height != this->height)
        {
            this->print_result(false);
            while(true);
        }
        else if (last_width != this->width && last_height != this->height)
        {
            throw std::runtime_error("Got invalid dimensions");
        }
    }

    void CursesWindow::get_row()
    {
        ssize_t size = recv(this->network_socket, this->row_buf, sizeof(block_t) * this->width, MSG_WAITALL);
        if (size == -1)
        {
            throw std::runtime_error("Failed receiving dim");
        }
        else if (size != static_cast<long long>(sizeof(block_t) * this->width))
        {
            throw std::runtime_error("Received the wrong number of bytes for dim");
        }
    }

    void CursesWindow::init_curses_app()
    {
        std::setvbuf(stdout, NULL, _IONBF, 0);

        char cmd[30] = {0};
        snprintf(cmd, 30, "eval `resize -s %u %u`", this->height + 2, this->width + 2);
        printf("Executing: \"%s\"", cmd);
        system(cmd);
        printf("Setting up curses\n");
        initscr(); // Initialize the curses screen

        resizeterm(this->height + 2, this->width + 2);

        if (LINES < static_cast<long>(this->height + 2))
        {
            throw std::runtime_error("Screen is not tall enough");
        }
        if (COLS < static_cast<long>(this->width + 2))
        {
            throw std::runtime_error("Screen is not wide enough");
        }
        noecho();             // Characters typed ar not echoed
        cbreak();             // No buffering of stdin
        nonl();               // Do not translate 'return key' on keyboard to newline character
        keypad(stdscr, TRUE); // Enable the keypad
        curs_set(0);          // Make cursor invisible
        // Begin in non-single-step mode (getch will not block)
        nodelay(stdscr, TRUE); // make getch to be a non-blocking call

        start_color();

        init_pair(EMPTY, COLOR_BLACK, COLOR_BLACK);
        init_pair(WALL, COLOR_RED, COLOR_BLACK);
        init_pair(PORTAL, COLOR_GREEN, COLOR_WHITE);
        init_pair(FLAG_A, COLOR_BLUE, COLOR_BLACK);
        init_pair(FLAG_B, COLOR_RED, COLOR_BLACK);
        init_pair(BASE_A, COLOR_BLACK, COLOR_BLUE);
        init_pair(BASE_B, COLOR_BLACK, COLOR_RED);
        init_pair(NOT_MAPPED, COLOR_WHITE, COLOR_WHITE);
        init_pair(WIN_TEXT, COLOR_WHITE, COLOR_GREEN);
        init_pair(LOOSE_TEXT, COLOR_WHITE, COLOR_RED);

        mvprintw(10, 10, "Hello from curses");

        attron(COLOR_PAIR(WALL));
        // Print upper and lower bounds
        for (size_t x = 0; x < this->width + 2; ++x)
        {
            mvaddch(0, x, '0');
            mvaddch(this->height + 1, x, '0');
        }
        // Print left and right bounds
        for (size_t y = 0; y < this->height + 2; ++y)
        {
            mvaddch(y, 0, '0');
            mvaddch(y, this->width + 1, '0');
        }
        attroff(COLOR_PAIR(WALL));
        refresh();
    }

    void CursesWindow::print_row()
    {
        for (size_t x = 0; x < this->width; ++x)
        {
            const block_t cur_block = this->row_buf[x];
            // Check if current block is an player
            if ((cur_block & (static_cast<block_t>(1) << (std::numeric_limits<block_t>::digits - 1))) != 0)
            {
                // init_pair(cur_player_index, COLOR_GREEN, COLOR_CYAN);
                short player_b = cur_block & ((1 << 10) - 1);         // lowest 10 bits
                short player_g = (cur_block & ((1 << 20) - 1)) >> 10; // The next 10 bits
                short player_r = (cur_block & ((1 << 30) - 1)) >> 20; // The next 10 bits
                player_b %= 1000;
                player_g %= 1000;
                player_r %= 1000;

                init_color(cur_player_index, player_r, player_g, player_b);

                if(!this->is_server) {
                    if ((cur_block & (static_cast<block_t>(1) << (std::numeric_limits<block_t>::digits - 4))) != 0) { // Check 4th MSB if this is the own player
                        if ((cur_block & (static_cast<block_t>(1) << (std::numeric_limits<block_t>::digits - 2))) != 0) { // Check 2nd MSB
                            init_pair(cur_player_index, COLOR_BLUE, COLOR_WHITE); // Team A
                        } else {
                            init_pair(cur_player_index, COLOR_RED, COLOR_WHITE); // Team B
                        }
                    } else {
                        if ((cur_block & (static_cast<block_t>(1) << (std::numeric_limits<block_t>::digits - 2))) != 0)
                        {                                                              // Check 2nd MSB
                            init_pair(cur_player_index, cur_player_index, COLOR_BLUE); // Team A
                        }
                        else
                        {
                            init_pair(cur_player_index, cur_player_index, COLOR_RED); // Team B
                        }
                    }
                } else {
                    if ((cur_block & (static_cast<block_t>(1) << (std::numeric_limits<block_t>::digits - 2))) != 0)
                    {                                                              // Check 2nd MSB
                        init_pair(cur_player_index, cur_player_index, COLOR_BLUE); // Team A
                    }
                    else
                    {
                        init_pair(cur_player_index, cur_player_index, COLOR_RED); // Team B
                    }
                }

                attron(COLOR_PAIR(this->cur_player_index));
                if((cur_block & (static_cast<block_t>(1) << (std::numeric_limits<block_t>::digits - 3))) != 0) {
                    mvaddch(this->cur_row + 1, x + 1, 'F');
                } else {
                    mvaddch(this->cur_row + 1, x + 1, 'P');
                }
                attroff(COLOR_PAIR(this->cur_player_index));
                this->cur_player_index++;
            }
            else
            {
                switch (cur_block)
                {
                case EMPTY:
                    attron(COLOR_PAIR(EMPTY));
                    mvaddch(this->cur_row + 1, x + 1, ' ');
                    attroff(COLOR_PAIR(EMPTY));
                    break;
                case WALL:
                    attron(COLOR_PAIR(WALL));
                    mvaddch(this->cur_row + 1, x + 1, '0');
                    attroff(COLOR_PAIR(WALL));
                    break;
                case PORTAL:
                    attron(COLOR_PAIR(PORTAL));
                    mvaddch(this->cur_row + 1, x + 1, '@');
                    attroff(COLOR_PAIR(PORTAL));
                    break;
                case FLAG_A:
                    attron(COLOR_PAIR(FLAG_A));
                    mvaddch(this->cur_row + 1, x + 1, '*');
                    attroff(COLOR_PAIR(FLAG_A));
                    break;
                case FLAG_B:
                    attron(COLOR_PAIR(FLAG_B));
                    mvaddch(this->cur_row + 1, x + 1, '*');
                    attroff(COLOR_PAIR(FLAG_B));
                    break;
                case BASE_A:
                    attron(COLOR_PAIR(BASE_A));
                    mvaddch(this->cur_row + 1, x + 1, '#');
                    attroff(COLOR_PAIR(BASE_A));
                    break;
                case BASE_B:
                    attron(COLOR_PAIR(BASE_B));
                    mvaddch(this->cur_row + 1, x + 1, '#');
                    attroff(COLOR_PAIR(BASE_B));
                    break;
                case NOT_MAPPED:
                    attron(COLOR_PAIR(NOT_MAPPED));
                    mvaddch(this->cur_row + 1, x + 1, ' ');
                    attroff(COLOR_PAIR(NOT_MAPPED));
                    break;
                default:
                    mvprintw(15, 5, "Invalid Block: %d", cur_block);
                    refresh();
                    throw std::runtime_error("Invalid block");
                    break;
                }
            }
        }
    }

    void CursesWindow::handle_update()
    {
        this->check_dim();
        this->cur_player_index = PLAYER;
        for (cur_row = 0; cur_row < this->height; ++cur_row)
        {
            this->get_row();
            this->print_row();
        }
        refresh();
    }

    void CursesWindow::print_result(const bool team_A)
    {
        attron(COLOR_PAIR(PORTAL));
        mvprintw(0,0, "GAME END!");
        attroff(COLOR_PAIR(PORTAL));
        if (this->is_server)
        {
            if (team_A)
            {
                const char *text = "TEAM A WON!";
                attron(COLOR_PAIR(WIN_TEXT));
                mvprintw((this->height + 2) / 2, (this->width + 2 - strlen(text)) / 2, "%s", text);
                attroff(COLOR_PAIR(WIN_TEXT));
                refresh();
            }
            else
            {
                const char *text = "TEAM B WON!";
                attron(COLOR_PAIR(WIN_TEXT));
                mvprintw((this->height + 2) / 2, (this->width + 2 - strlen(text)) / 2, "%s", text);
                attroff(COLOR_PAIR(WIN_TEXT));
                refresh();
            }
        }
        else if (this->team_A)
        {
            if (team_A)
            {
                const char *text = "SUCCESS TEAM A WON!";
                attron(COLOR_PAIR(WIN_TEXT));
                mvprintw((this->height + 2) / 2, (this->width + 2 - strlen(text)) / 2, "%s", text);
                attroff(COLOR_PAIR(WIN_TEXT));
                refresh();
            }
            else
            {
                const char *text = "FAIL TEAM A LOST!";
                attron(COLOR_PAIR(LOOSE_TEXT));
                mvprintw((this->height + 2) / 2, (this->width + 2 - strlen(text)) / 2, "%s", text);
                attroff(COLOR_PAIR(LOOSE_TEXT));
                refresh();
            }
        }
        else
        {
            if (!team_A)
            {
                const char *text = "SUCCESS TEAM B WON!";
                attron(COLOR_PAIR(WIN_TEXT));
                mvprintw((this->height + 2) / 2, (this->width + 2 - strlen(text)) / 2, "%s", text);
                attroff(COLOR_PAIR(WIN_TEXT));
                refresh();
            }
            else
            {
                const char *text = "FAIL TEAM B LOST!";
                attron(COLOR_PAIR(LOOSE_TEXT));
                mvprintw((this->height + 2) / 2, (this->width + 2 - strlen(text)) / 2, "%s", text);
                attroff(COLOR_PAIR(LOOSE_TEXT));
                refresh();
            }
        }

        refresh();
    }
}

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        throw std::runtime_error("Please provide a port to listen on for incoming data an the team [A,B,S]");
    }
    bool is_server = false;
    bool team_A = true;
    if (argv[2][0] == 'S')
    {
        is_server = true;
    }
    else if (argv[2][0] == 'A')
    {
        team_A = true;
    }
    else if (argv[2][0] == 'B')
    {
        team_A = false;
    }
    else
    {
        throw std::runtime_error("Invalid team parameter");
    }
    jps_maze_curses::CursesWindow curses_window(argv[1], is_server, team_A);
    while (true)
    {
        curses_window.handle_update();
    }
    return EXIT_SUCCESS;
}