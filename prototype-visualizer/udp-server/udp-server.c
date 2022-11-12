#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <time.h>

#include <sys/types.h>
#include <sys/socket.h>

#include <netinet/in.h>

#define ARENA_HEIGHT 64
#define ARENA_WIDTH 48

int arena_row[ARENA_WIDTH];
// create a udp socket
int network_socket;
network_socket = socket(AF_INET, SOCK_DGRAM, 0);

// address for socket
struct sockaddr_in server_address;
server_address.sin_family = AF_INET; 
server_address.sin_port = htons(42069);
inet_aton ("127.0.0.1", &server_address.sin_addr);

void sendRandomArena()
{

    for(int i = 0; i < ARENA_HEIGHT; i++)
    {

        for(int n = 0; n < ARENA_WIDTH; n++)
        {

            arena[i][n] = rand() % 10;
            sendto(network_socket, arena_row, sizeof(arena_row), 0, (const struct sockaddr *) &server_address, sizeof(server_address));

        }

    }

    return;
}

int main()
{
    
    srand(time(NULL)); 

    // bind address to socket
    //TODO: make some error handling, but screw that, I am a consultant 
    bind(network_socket, (struct sockaddr*) &server_address, sizeof(server_address));

    int widthheight[] = {ARENA_WIDTH, ARENA_HEIGHT};

    sendto(network_socket, widthheight, sizeof(widthheight), 0, (const struct sockaddr *) &server_address, sizeof(server_address));

    //smash it
    while(true)
    {
        sendRandomArena();
        printf("Sended!\n");
        sleep(3);
    }

    return 0;

}