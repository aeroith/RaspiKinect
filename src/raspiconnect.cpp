/*** TODO: Add exceptions instead of C style error catch ***/

// Good chunk size
#define MAX_XFER_BUF_SIZE 16384

#include "raspiconnect.h"
#include <fcntl.h>
#include <sys/stat.h>

int RaspberryCon::verify_knownhost(ssh_session session) {
    int state, hlen;
    unsigned char *hash = NULL;
    char *hexa;
    char buf[10];
    state = ssh_is_server_known(session);
    hlen = ssh_get_pubkey_hash(session, &hash);
    if (hlen < 0)
        return -1;
    switch (state) {
    case SSH_SERVER_KNOWN_OK:
        break; /* ok */
    case SSH_SERVER_KNOWN_CHANGED:
        fprintf(stderr, "Host key for server changed: it is now:\n");
        ssh_print_hexa("Public key hash", hash, hlen);
        fprintf(stderr, "For security reasons, connection will be stopped\n");
        free(hash);
        return -1;
    case SSH_SERVER_FOUND_OTHER:
        fprintf(stderr, "The host key for this server was not found but an other"
                        "type of key exists.\n");
        fprintf(stderr,
                "An attacker might change the default server key to"
                "confuse your client into thinking the key does not exist\n");
        free(hash);
        return -1;
    case SSH_SERVER_FILE_NOT_FOUND:
        fprintf(stderr, "Could not find known host file.\n");
        fprintf(stderr, "If you accept the host key here, the file will be"
                        "automatically created.\n");
        /* fallback to SSH_SERVER_NOT_KNOWN behavior */
    case SSH_SERVER_NOT_KNOWN:
        hexa = ssh_get_hexa(hash, hlen);
        fprintf(stderr, "The server is unknown. Do you trust the host key?\n");
        fprintf(stderr, "Public key hash: %s\n", hexa);
        free(hexa);
        if (fgets(buf, sizeof(buf), stdin) == NULL) {
            free(hash);
            return -1;
        }
        if (strncasecmp(buf, "yes", 3) != 0) {
            free(hash);
            return -1;
        }
        if (ssh_write_knownhost(session) < 0) {
            fprintf(stderr, "Error %s\n", strerror(errno));
            free(hash);
            return -1;
        }
        break;
    case SSH_SERVER_ERROR:
        fprintf(stderr, "Error %s", ssh_get_error(session));
        free(hash);
        return -1;
    }
    free(hash);
    return 0;
}
void RaspberryCon::init(std::string user, std::string address,
                        std::string password) {
    ssh_session raspberry;
    _hostname = user + "@" + address;
    _password = password;
    // Open session and set options
    raspberry = ssh_new();
    if (raspberry == NULL)
        exit(-1);
    ssh_options_set(raspberry, SSH_OPTIONS_HOST, _hostname.c_str());
    _raspberry = raspberry;
}
int RaspberryCon::connect() {
    // Connect to server
    ssh_session raspberry = _raspberry;
    // char *password = _password.c_str();
    int rc;
    rc = ssh_connect(raspberry);
    if (rc != SSH_OK) {
        fprintf(stderr, "Error connecting to host: %s\n", ssh_get_error(raspberry));
        ssh_free(raspberry);
        exit(-1);
    }
    // Verify the server's identity
    if (verify_knownhost(raspberry) < 0) {
        ssh_disconnect(raspberry);
        ssh_free(raspberry);
        exit(-1);
    }
    // Authenticate ourselves
    // password = getpass("Password: ");
    rc = ssh_userauth_password(raspberry, NULL, _password.c_str());
    if (rc != SSH_AUTH_SUCCESS) {
        fprintf(stderr, "Error authenticating with password: %s\n",
                ssh_get_error(raspberry));
        ssh_disconnect(raspberry);
        ssh_free(raspberry);
        exit(-1);
    }
}
int RaspberryCon::runKinect(std::string depthfile = "depth",
                            std::string rgbfile = "rgb") {
    ssh_session session = _raspberry;
    ssh_channel channel;
    _depthfile = depthfile;
    _rgbfile = rgbfile;
    int rc;
    char buffer[256];
    int nbytes;
    channel = ssh_channel_new(session);
    if (channel == NULL)
        return SSH_ERROR;
    rc = ssh_channel_open_session(channel);
    if (rc != SSH_OK) {
        ssh_channel_free(channel);
        return rc;
    }
    std::string command = "./kinectRun " + _depthfile + " " + _rgbfile;
    rc = ssh_channel_request_exec(channel, command.c_str());
    if (rc != SSH_OK) {
        ssh_channel_close(channel);
        ssh_channel_free(channel);
        return rc;
    }
    nbytes = ssh_channel_read(channel, buffer, sizeof(buffer), 0);
    while (nbytes > 0) {
        if (write(1, buffer, nbytes) != (unsigned int)nbytes) {
            ssh_channel_close(channel);
            ssh_channel_free(channel);
            return SSH_ERROR;
        }
        nbytes = ssh_channel_read(channel, buffer, sizeof(buffer), 0);
    }

    if (nbytes < 0) {
        ssh_channel_close(channel);
        ssh_channel_free(channel);
        return SSH_ERROR;
    }
    ssh_channel_send_eof(channel);
    ssh_channel_close(channel);
    ssh_channel_free(channel);
    return SSH_OK;
}

// get rgb and depth data from raspberry pi
int RaspberryCon::getData() {
    ssh_session session = _raspberry;
    sftp_session sftp;
    int rc;
    sftp = sftp_new(session);
    if (sftp == NULL) {
        fprintf(stderr, "Error allocating SFTP session: %s\n",
                ssh_get_error(session));
        return SSH_ERROR;
    }
    rc = sftp_init(sftp);
    if (rc != SSH_OK) {
        fprintf(stderr, "Error initializing SFTP session: %s.\n",
                sftp_get_error(sftp));
        sftp_free(sftp);
        return rc;
    }
    sftp_read_sync(session, sftp);
    sftp_free(sftp);
    return SSH_OK;
}

int RaspberryCon::sftp_read_sync(ssh_session session, sftp_session sftp) {
    int access_type;
    sftp_file file;
    uint8_t buffer[MAX_XFER_BUF_SIZE];
    int nbytes, nwritten, rc;
    int fd;
    access_type = O_RDONLY;
    file = sftp_open(sftp, _rgbfile.c_str(), access_type, 0);
    if (file == NULL) {
        fprintf(stderr, "Can't open file for reading: %s\n",
                ssh_get_error(session));
        return SSH_ERROR;
    }
    fd = open(_rgbfile.c_str(), O_CREAT | O_WRONLY);

    if (fd < 0) {
        fprintf(stderr, "Can't open file for writing: %s\n", strerror(errno));
        return SSH_ERROR;
    }
    for (;;) {
        nbytes = sftp_read(file, buffer, sizeof(buffer));
        if (nbytes == 0) {
            break; // EOF
        } else if (nbytes < 0) {
            fprintf(stderr, "Error while reading file: %s\n", ssh_get_error(session));
            sftp_close(file);
            return SSH_ERROR;
        }
        nwritten = write(fd, buffer, nbytes);
        if (nwritten != nbytes) {
            fprintf(stderr, "Error writing: %s\n", strerror(errno));
            sftp_close(file);
            return SSH_ERROR;
        }
    }
    rc = sftp_close(file);
    if (rc != SSH_OK) {
        fprintf(stderr, "Can't close the read file: %s\n", ssh_get_error(session));
        return rc;
    }
    file = sftp_open(sftp, _depthfile.c_str(), access_type, 0);
    if (file == NULL) {
        fprintf(stderr, "Can't open file for reading: %s\n",
                ssh_get_error(session));
        return SSH_ERROR;
    }
    fd = open(_depthfile.c_str(), O_CREAT | O_WRONLY);

    if (fd < 0) {
        fprintf(stderr, "Can't open file for writing: %s\n", strerror(errno));
        return SSH_ERROR;
    }
    for (;;) {
        nbytes = sftp_read(file, buffer, sizeof(buffer));
        if (nbytes == 0) {
            break; // EOF
        } else if (nbytes < 0) {
            fprintf(stderr, "Error while reading file: %s\n", ssh_get_error(session));
            sftp_close(file);
            return SSH_ERROR;
        }
        nwritten = write(fd, buffer, nbytes);
        if (nwritten != nbytes) {
            fprintf(stderr, "Error writing: %s\n", strerror(errno));
            sftp_close(file);
            return SSH_ERROR;
        }
    }
    rc = sftp_close(file);
    if (rc != SSH_OK) {
        fprintf(stderr, "Can't close the read file: %s\n", ssh_get_error(session));
        return rc;
    }
    return SSH_OK;
}

void RaspberryCon::disconnect() {
    ssh_disconnect(_raspberry);
    ssh_free(_raspberry);
}
