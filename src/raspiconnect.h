#pragma once
#define LIBSSH_STATIC 1
#include <errno.h>
#include <iostream>
#include <libssh/libssh.h>
#include <libssh/sftp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>

class RaspberryCon {
public:
    RaspberryCon() = default;
    void init(std::string user, std::string address,
              std::string password); // initialize connection parameters
    int connect();                   // connect to raspberry pi
    int runKinect(std::string depthfile,
                  std::string rgbfile); // run the kinect to get specified data
    int getData();                      // read data to localhost
    void disconnect();

private:
    int sftp_read_sync(ssh_session session, sftp_session sftp);
    int verify_knownhost(ssh_session);
    std::string _hostname;
    std::string _password;
    ssh_session _raspberry;
    std::string _depthfile;
    std::string _rgbfile;
    int _rc;
};
