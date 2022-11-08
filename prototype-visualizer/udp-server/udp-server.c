#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>

#include <sys/types.h>
#include <sys/socket.h>

#include <netinet/in.h>

#define ARENA_SIZE 64

int main()
{
    //initialize and fill dummy arena
    int arena[ARENA_SIZE][ARENA_SIZE];

    int k = 0;

    for(int i = 0; i < ARENA_SIZE; i++)
    {

        for(int n = 0; n < ARENA_SIZE; n++)
        {

            arena[i][n] = k;
            k++;
            if(k>4)k=0;

        }

    }

    // create a udp socket
    int network_socket;
    network_socket = socket(AF_INET, SOCK_DGRAM, 0);

    // address for socket
    struct sockaddr_in server_address;
    server_address.sin_family = AF_INET; 
    server_address.sin_port = htons(42069);
    inet_aton ("127.0.0.1", &server_address.sin_addr);

    // bind address to socket
    //TODO: make some error handling, but screw that, I am a consultant 
    bind(network_socket, (struct sockaddr*) &server_address, sizeof(server_address));

    //smash it
    while(true)
    {
        sendto(network_socket, arena, sizeof(arena), 0, (const struct sockaddr *) &server_address, sizeof(server_address));
        printf("Sended!\n");
        sleep(3);
    }

    return 0;

}