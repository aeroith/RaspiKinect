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

/**
* This class contains the functions to access the kinects
* using Raspberry Pi
*/
class RaspberryCon {
public:
    //
    // default constructor
    //
    RaspberryCon() = default;

    /**
    * Initialize the connection parameters.
    * @param[in] user hostname of rasperry pi
    * @param[in] password password of raspberry pi
    * @param[in] address assigned IP address of raspberry pi
    */
    void init(std::string user, std::string address,
              std::string password); 

    /**
    * Connect to raspberry pi
    */
    int connect();

    /**
    * Run the kinect connected to raspberry pi. Make sure the
    * names of depth and rgb files are unique to each Pi.
    * @param[in] depthfile name of the acquired depth data
    * @param[in] rgbfile name of the acquired rgb data
    */
    int runKinect(std::string depthfile,
                  std::string rgbfile); 
    /**
    * Read data to localhost. The name of the data is the same
    * that is selected in runKinect() function
    */
    int getData();

    /**
    * Disconnect from Raspberry Pi. Frees the memory reserved for
    * SSH session.
    */
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
