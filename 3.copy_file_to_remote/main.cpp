#include <iostream>
#include <libssh2.h>
#include <libssh2_sftp.h>

int main() {
    // 初始化libssh2库
    int rc = libssh2_init(0);
    if (rc != 0) {
        std::cerr << "libssh2 initialization failed." << std::endl;
        return 1;
    }

    // 建立SSH会话
    LIBSSH2_SESSION *session = libssh2_session_init();
    if (!session) {
        std::cerr << "Unable to initialize SSH session." << std::endl;
        libssh2_exit();
        return 1;
    }

    // 设置SSH连接信息
    const char *hostname = "192.168.8.182";
    const char *username = "xdan";
    const char *password = "123";
    const int port = 22;

    int sock = socket(AF_INET, SOCK_STREAM, 0);

    // 连接到远程主机
    if (libssh2_session_startup(session, 0) < 0) {
        std::cout << "sftp_session: " << libssh2_session_startup(session, 0)  << std::endl;
        std::cerr << "Unable to establish SSH session." << std::endl;
        libssh2_session_free(session);
        libssh2_exit();
        return 1;
    }

    // 认证
    if (libssh2_userauth_password(session, username, password) < 0) {
        std::cerr << "Authentication failed." << std::endl;
        libssh2_session_free(session);
        libssh2_exit();
        return 1;
    }

    // 初始化SFTP会话
    LIBSSH2_SFTP *sftp_session = libssh2_sftp_init(session);
    if (!sftp_session) {
        std::cerr << "Unable to initialize SFTP session." << std::endl;
        libssh2_session_disconnect(session, "Normal Shutdown");
        libssh2_session_free(session);
        libssh2_exit();
        return 1;
    }

    // 本地文件路径
    const char *local_file_path = "/home/wzj/wzj/repos/python_test/read_remote_J5_image/images/1.png";

    // 远程文件路径
    const char *remote_file_path = "C:/Users/xdan/Desktop/cat.png";

    // 打开本地文件
    FILE *local_file = fopen(local_file_path, "rb");
    if (!local_file) {
        std::cerr << "Unable to open local file." << std::endl;
        libssh2_sftp_shutdown(sftp_session);
        libssh2_session_disconnect(session, "Normal Shutdown");
        libssh2_session_free(session);
        libssh2_exit();
        return 1;
    }

    // 创建远程文件
    LIBSSH2_SFTP_HANDLE *sftp_handle = libssh2_sftp_open(sftp_session, remote_file_path,
                                                        LIBSSH2_FXF_WRITE | LIBSSH2_FXF_CREAT | LIBSSH2_FXF_TRUNC,
                                                        LIBSSH2_SFTP_S_IRUSR | LIBSSH2_SFTP_S_IWUSR |
                                                        LIBSSH2_SFTP_S_IRGRP | LIBSSH2_SFTP_S_IROTH);
    if (!sftp_handle) {
        std::cerr << "Unable to open remote file." << std::endl;
        fclose(local_file);
        libssh2_sftp_shutdown(sftp_session);
        libssh2_session_disconnect(session, "Normal Shutdown");
        libssh2_session_free(session);
        libssh2_exit();
        return 1;
    }

    // 读取本地文件内容并写入远程文件
    char buffer[4096];
    size_t nread;
    while ((nread = fread(buffer, 1, sizeof(buffer), local_file)) > 0) {
        ssize_t nwritten = libssh2_sftp_write(sftp_handle, buffer, nread);
        if (nwritten != static_cast<ssize_t>(nread)) {
            std::cerr << "Error writing to remote file." << std::endl;
            libssh2_sftp_close(sftp_handle);
            fclose(local_file);
            libssh2_sftp_shutdown(sftp_session);
            libssh2_session_disconnect(session, "Normal Shutdown");
            libssh2_session_free(session);
            libssh2_exit();
            return 1;
        }
    }

    // 关闭文件句柄和文件
    libssh2_sftp_close(sftp_handle);
    fclose(local_file);

    // 关闭SFTP会话
    libssh2_sftp_shutdown(sftp_session);

    // 断开SSH连接
    libssh2_session_disconnect(session, "Normal Shutdown");
    libssh2_session_free(session);

    // 退出libssh2库
    libssh2_exit();

    std::cout << "File transfer complete." << std::endl;

    return 0;
}
