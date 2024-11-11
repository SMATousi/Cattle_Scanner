#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <unistd.h>

#define PORT 4210 // Local UDP Port
#define BROADCAST_IP "255.255.255.255"
#define BROADCAST_INTERVAL 2 // Interval time in seconds for each message broadcast

int main(){

	int sockfd;
	struct sockaddr_in broadcastAddr;
	int message = 0;
	int broadcastPermission = 1;

	char userInput;

	// Create Socket
	if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
		perror("socker creation failed");
		exit(EXIT_FAILURE);
	}


	// Set socket options to allow broadcast
	if (setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &broadcastPermission, sizeof(broadcastPermission)) < 0)
	{
		perror("setsockopt failed");
		close(sockfd);
		exit(EXIT_FAILURE);
	}


	memset(&broadcastAddr, 0, sizeof(broadcastAddr));
	broadcastAddr.sin_family = AF_INET;
	broadcastAddr.sin_port = htons(PORT);
	broadcastAddr.sin_addr.s_addr = inet_addr(BROADCAST_IP);

while(1){
		// Send broadcast message
		printf("Enter 1 to broadcast message or 0 to exit: ");
		userInput = getchar();

		// Clear the newline character from the input buffer
		while(getchar() != '\n');

		if (userInput == '1'){

			if (sendto(sockfd, &message, sizeof(message), 0, (struct sockaddr*)&broadcastAddr, sizeof(broadcastAddr)) < 0 ) {
				perror("sendto failed");
				close(sockfd);
				exit(EXIT_FAILURE);
			}
			printf("Broadcast message sent: %d.\n", message);
			message++; // Increment message for next broadcast
		}
		else if(userInput == '0'){
			printf("Exiting program.\n");
			break;
		}
		else {
			printf("Invalid input. Please enter 1 or 0.\n");
		}

			// Wait for specified interval
			//sleep(BROADCAST_INTERVAL);
	}

	close(sockfd);
	return 0;
}
