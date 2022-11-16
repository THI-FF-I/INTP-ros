#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <time.h>

#include <sys/types.h>
#include <sys/socket.h>

#include <netinet/in.h>

#define ARENA_HEIGHT 123
#define ARENA_WIDTH 142

int arena_row[ARENA_WIDTH];

// create a udp socket
int network_socket;

// address for socket
struct sockaddr_in server_address;

int widthheight[] = {ARENA_WIDTH, ARENA_HEIGHT};

void sendRandomArena()
{
    
    sendto(network_socket, widthheight, sizeof(widthheight), 0, (const struct sockaddr *) &server_address, sizeof(server_address));

    for(int i = 0; i < ARENA_HEIGHT; i++)
    {

        for(int n = 0; n < ARENA_WIDTH; n++)
        {
            arena_row[n] = rand() % 9;

        }

        sendto(network_socket, arena_row, sizeof(arena_row), 0, (const struct sockaddr *) &server_address, sizeof(server_address));

    }

    return;
}

int main()
{
    
    int i = 0;

    srand(time(NULL));
    network_socket = socket(AF_INET, SOCK_DGRAM, 0);
    server_address.sin_family = AF_INET; 
    server_address.sin_port = htons(42069);
    inet_aton ("127.0.0.1", &server_address.sin_addr);

    // bind address to socket
    //TODO: make some error handling, but screw that, I am a consultant 
    bind(network_socket, (struct sockaddr*) &server_address, sizeof(server_address));

    //smash it
    while(true)
    {
        sendRandomArena();
        printf("Sended!\n");
        sleep(1);
        i++;
        if(i > 2) return 0;
    }

    return 0;

}