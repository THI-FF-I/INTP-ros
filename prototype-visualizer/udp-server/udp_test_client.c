#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include <sys/types.h>
#include <sys/socket.h>

#include <netinet/in.h>

#define ARENA_SIZE 64

int main()
{
    //initialize dummy arena
    int arena[ARENA_SIZE][ARENA_SIZE];

    // create a udp socket
    int network_socket;
    network_socket = socket(AF_INET, SOCK_DGRAM, 0);

    // address for socket
    struct sockaddr_in server_address;
    server_address.sin_family = AF_INET; 
    server_address.sin_port = htons(42069);
    server_address.sin_addr.s_addr = INADDR_ANY;

    // bind address to socket
    //TODO: make some error handling, but screw that, I am a consultant 
    bind(network_socket, (struct sockaddr*) &server_address, sizeof(server_address));

    //smash it
    while(true)
    {
        recvfrom(network_socket, &arena, sizeof(arena), MSG_WAITALL, 0, 0);
        
        for(int i = 0; i < 64; i++)
        {

            for(int k = 0; k < 64; k++)
            {
                printf("%d", arena[i][k]);
            }

            printf("\n");

        }
        
    }

    return 0;

}