#include <arpa/inet.h>
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <thread>

struct Data {
  int id;
  float value;
  char name[32];
};

int main() {
  int sock = socket(AF_INET, SOCK_DGRAM, 0);

  sockaddr_in server_addr{};
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(8888);
  inet_pton(AF_INET, "127.0.0.1", &server_addr.sin_addr);

  Data d = {42, 3.14f, "HelloUDP"};

  while (true) {
    sendto(sock, reinterpret_cast<char*>(&d), sizeof(d), 0,
           (sockaddr*)&server_addr, sizeof(server_addr));

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    /* code */
  }

  std::cout << "Sent struct\n";
  close(sock);
  return 0;
}