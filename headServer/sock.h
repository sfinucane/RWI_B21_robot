typedef struct sockaddr SA;

ssize_t writen(int fd, const void *vptr, size_t n);

ssize_t readn(int fd, void *vptr, size_t n);

int open_tcp_server(int port);

int listen_for_connection(int listenfd);

void close_connection(int sock);

int connect_to_server(char *host, int port);
