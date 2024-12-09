#include <libssh2.h>
#include <libssh2_sftp.h>

#include <iostream>
#include <fstream>

int main() {
    // 初始化libssh2库
    int rc = libssh2_init(0);
    if (rc != 0) {
        std::cerr << "Unable to initialize libssh2." << std::endl;
        return 1;
    }

    // 创建SSH会话
    LIBSSH2_SESSION *session;
    session = libssh2_session_init();
    if (!session) {
        std::cerr << "Unable to create session." << std::endl;
        return 1;
    }

    // 设置SSH会话的选项
    libssh2_session_set_blocking(session, 1);
    
    // 连接到远程服务器
    const char *hostname = "192.168.8.182";
    const char *username = "xdan";
    const char *password = "123";
    const int port = 22;

    if (libssh2_session_startup(session, -1) != 0) {
        std::cerr << "Failed to establish SSH session." << std::endl;
        return 1;
    }

    // 使用用户名和密码进行认证
    if (libssh2_userauth_password(session, username, password) != 0) {
        std::cerr << "Authentication failed." << std::endl;
        return 1;
    }

    // 打开SFTP会话
    LIBSSH2_SFTP *sftp_session;
    sftp_session = libssh2_sftp_init(session);
    if (!sftp_session) {
        std::cerr << "Unable to initialize SFTP session." << std::endl;
        return 1;
    }

    // 本地文件路径
    const char *local_path = "/home/wzj/wzj/repos/python_test/read_remote_J5_image/images/1.png";

    // 远程文件路径
    const char *remote_path = "C:/Users/xdan/Desktop/cat.png";

    // 打开本地文件
    std::ifstream local_file(local_path, std::ios::binary);
    if (!local_file.is_open()) {
        std::cerr << "Failed to open local file." << std::endl;
        return 1;
    }

    // 创建远程文件
    LIBSSH2_SFTP_HANDLE *sftp_handle;
    sftp_handle = libssh2_sftp_open(sftp_session, remote_path, LIBSSH2_FXF_WRITE | LIBSSH2_FXF_CREAT | LIBSSH2_FXF_TRUNC,
                                    LIBSSH2_SFTP_S_IRUSR | LIBSSH2_SFTP_S_IWUSR | LIBSSH2_SFTP_S_IRGRP | LIBSSH2_SFTP_S_IROTH);
    if (!sftp_handle) {
        std::cerr << "Failed to open remote file." << std::endl;
        return 1;
    }

    // 传输文件内容
    char buffer[1024];
    int bytes_read;
    while ((bytes_read = local_file.readsome(buffer, sizeof(buffer))) > 0) {
        if (libssh2_sftp_write(sftp_handle, buffer, bytes_read) != bytes_read) {
            std::cerr << "Failed to write to remote file." << std::endl;
            return 1;
        }
    }

    // 关闭文件句柄和SFTP会话
    libssh2_sftp_close(sftp_handle);
    libssh2_sftp_shutdown(sftp_session);

    // 关闭本地文件
    local_file.close();

    // 关闭SSH会话
    libssh2_session_disconnect(session, "Normal Shutdown");
    libssh2_session_free(session);

    // 关闭libssh2库
    libssh2_exit();

    return 0;
}

