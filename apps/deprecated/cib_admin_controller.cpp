/*
 * test_daq_client.cpp
 *
 *  Created on: Mar 24, 2024
 *      Author: Nuno Barros
 */


#include <iostream>
#include <zmq.hpp>
#include <sstream>
#include "cib.pb.h"
#include <cstdio>
// necessary to have getopt
extern "C"
{
#include <unistd.h>
};

#include <readline/readline.h>
#include <readline/history.h>

void print_usage(const char *prog) {
    printf( "Usage: %s [-w ip] [cmd] \n", prog);
}

void print_help() {
    printf("Available commands:\n");
    printf("  reboot\n");
    printf("    Reboot the WIB\n");
    printf("  fw_timestamp\n");
    printf("    Return the firmware version timestamp\n");
    printf("  sw_version\n");
    printf("    Return the software build version\n");
    printf("  log [boot|clear]\n");
    printf("    Return the wib_server log (or return boot log, or clear the logs)\n");
    printf("  script filename\n");
    printf("    Run a WIB script (local file will be sent, otherwise filename is remote in /etc/wib/)\n");
    printf("  daqspy filename\n");
    printf("    Read 1MB from each daq spy buffer and write the 2MB data to filename\n");
    printf("  peek addr\n");
    printf("    Read a 32bit value from WIB address space\n");
    printf("  poke addr value\n");
    printf("    Write a 32bit value to WIB address space\n");
    printf("  cdpeek femb_idx cd_idx chip_addr reg_page reg_addr\n");
    printf("    Read a 8bit value from COLDATA I2C address space\n");
    printf("  cdpoke femb_idx cd_idx chip_addr reg_page reg_addr data\n");
    printf("    Write a 8bit value to COLDATA I2C address space\n");
    printf("  cdfastcmd cmd\n");
    printf("    Send the fast command cmd to all coldata chips\n");
    printf("  update root_archive boot_archive\n");
    printf("    Deploy a new root and boot archive to the WIB\n");
    printf("  exit\n");
    printf("    Closes the command interface\n");
}

template <class R, class C>
void send_command(zmq::socket_t &socket, const C &msg, R &repl) {

    cib::Request request;
    request.mutable_request()->PackFrom(msg);

    std::string cmd_str;
    request.SerializeToString(&cmd_str);

    zmq::message_t req(cmd_str.size());
    memcpy((void*)req.data(), cmd_str.c_str(), cmd_str.size());
    socket.send(req,zmq::send_flags::none);

    zmq::message_t reply;
    socket.recv(reply,zmq::recv_flags::none);

    std::string reply_str(static_cast<char*>(reply.data()), reply.size());
    repl.ParseFromString(reply_str);
}

void send_config()
{

}

int run_command(zmq::socket_t &s, int argc, char **argv)
{
    if (argc < 1) return 1;

    std::string cmd(argv[0]);
    if (cmd == "exit") {
        return 255;
    }
    else if (cmd == "config")
    {
      cib::Config req;
      req.set_packet_size(0xFF);
      req.set_enable_monitor(false);
      req.set_stream_host("127.0.0.1");
      req.set_stream_port(5552);
      cib::Statistics rep;
      send_command(s,req,rep);
      std::string r;

      printf("Received reply with : [%li]\n",rep.ByteSizeLong());
//      wib::GetSensors req;
//      wib::GetSensors::Sensors rep;
//      send_command(s,req,rep);
//    } else if (cmd == "fw_timestamp") {
//        wib::GetTimestamp req;
//        wib::GetTimestamp::Timestamp rep;
//        send_command(s,req,rep);
//        printf("fw_timestamp code: 0x%08X\n",rep.timestamp());
//        printf("decoded: %i/%i/%i %i:%i:%i\n",rep.year(),rep.month(),rep.day(),rep.hour(),rep.min(),rep.sec());
//    } else if (cmd == "sw_version") {
//        wib::GetSWVersion req;
//        wib::GetSWVersion::Version rep;
//        send_command(s,req,rep);
//        printf("sw_version: %s\n",rep.version().c_str());
//    } else if (cmd == "log") {
//        if (argc > 2) {
//            printf("Usage: log [clear|boot]\n");
//            return 0;
//        }
//        bool boot = false;
//        bool clear = false;
//        if (argc == 2) {
//            string subcmd(argv[1]);
//            if (subcmd == "boot") {
//                boot = true;
//            } else if (subcmd == "clear") {
//                clear = true;
//            } else {
//                printf("Usage: log [clear|boot]\n");
//                return 0;
//            }
//        }
//        wib::LogControl req;
//        req.set_return_log(!boot && !clear);
//        req.set_boot_log(boot);
//        req.set_clear_log(clear);
//        wib::LogControl::Log rep;
//        send_command(s,req,rep);
//        const string &log = rep.contents();
//        printf("%s",log.c_str());
//    } else if (cmd == "script") {
//        if (argc != 2) {
//            printf("Usage: script filename\n");
//            return 0;
//        }
//        string fname(argv[1]);
//        ifstream fin(fname);
//        if (fin.is_open()) {
//            printf("Executing local script... ");
//            fflush(stdout);
//            string script((istreambuf_iterator<char>(fin)), istreambuf_iterator<char>());
//            wib::Script req;
//            req.set_script(script);
//            req.set_file(false);
//            wib::Status rep;
//            send_command(s,req,rep);
//            if (rep.success()) {
//                printf("Success\n");
//            } else {
//                printf("Failure\n");
//            }
//            fin.close();
//        } else {
//            printf("Executing remote script... ");
//            fflush(stdout);
//            wib::Script req;
//            req.set_script(fname);
//            req.set_file(true);
//            wib::Status rep;
//            send_command(s,req,rep);
//            if (rep.success()) {
//                printf("Success\n");
//            } else {
//                printf("Failure\n");
//            }
//        }
//    } else if (cmd == "daqspy") {
//        if (argc != 2) {
//            printf("Usage: daqspy filename\n");
//            return 0;
//        }
//        wib::ReadDaqSpy req;
//        req.set_buf0(true);
//        req.set_buf1(true);
//        wib::ReadDaqSpy::DaqSpy rep;
//        printf("Acquiring DAQ spy buffer...");
//        fflush(stdout);
//        send_command(s,req,rep);
//        string fname(argv[1]);
//        ofstream fout(fname,ofstream::binary);
//        fout.write(rep.buf0().c_str(),rep.buf0().size());
//        fout.write(rep.buf1().c_str(),rep.buf1().size());
//        fout.close();
//        if (rep.success()) {
//            printf("Success\n");
//        } else {
//            printf("Failure\n");
//        }
//    } else if (cmd == "calibrate") {
//        if (argc != 1) {
//            printf("Usage: calibrate\n");
//            return 0;
//        }
//        wib::Calibrate req;
//        wib::Status rep;
//        send_command(s,req,rep);
//        if (rep.success()) {
//            printf("Success\n");
//        } else {
//            printf("Failure\n");
//        }
//    } else if (cmd == "update") {
//        if (argc != 3) {
//            printf("Usage: update root_archive boot_archive\n");
//            return 0;
//        }
//
//        size_t length;
//        string root_archive, boot_archive;
//
//        ifstream in_root(argv[1], ios::binary);
//        if (!in_root.is_open()) {
//            printf("Could not open root archive: %s\n",argv[1]);
//            return 0;
//        }
//        in_root.ignore( numeric_limits<streamsize>::max() );
//        length = in_root.gcount();
//        in_root.clear();
//        in_root.seekg( 0, ios_base::beg );
//        root_archive.resize(length);
//        in_root.read((char*)root_archive.data(),length);
//        in_root.close();
//
//        ifstream in_boot(argv[2], ios::binary);
//        if (!in_boot.is_open()) {
//            printf("Could not open boot archive: %s\n",argv[2]);
//            return 0;
//        }
//        in_boot.ignore( numeric_limits<streamsize>::max() );
//        length = in_boot.gcount();
//        in_boot.clear();
//        in_boot.seekg( 0, ios_base::beg );
//        boot_archive.resize(length);
//        in_boot.read((char*)boot_archive.data(),length);
//        in_boot.close();
//
//        wib::Update req;
//        printf("Sending root archive (%0.1f MB) and boot archive (%0.1f MB)\n",root_archive.size()/1024.0/1024.0,boot_archive.size()/1024.0/1024.0);
//        req.set_root_archive(root_archive);
//        req.set_boot_archive(boot_archive);
//        wib::Empty rep;
//        send_command(s,req,rep);
//    } else if (cmd == "reboot") {
//        wib::Reboot req;
//        wib::Empty rep;
//        send_command(s,req,rep);
//    } else if (cmd == "peek") {
//        if (argc != 2) {
//            printf("Usage: peek addr\n");
//            printf("   all arguments are base 16\n");
//            return 0;
//        }
//        wib::Peek req;
//        req.set_addr((size_t)strtoull(argv[1],NULL,16));
//        wib::RegValue rep;
//        send_command(s,req,rep);
//        printf("0x%X\n",rep.value());
//    } else if (cmd == "poke") {
//        if (argc != 3) {
//            printf("Usage: poke addr value\n");
//            printf("   all arguments are base 16\n");
//            return 0;
//        }
//        wib::Poke req;
//        req.set_addr((size_t)strtoull(argv[1],NULL,16));
//        req.set_value((uint32_t)strtoull(argv[2],NULL,16));
//        wib::RegValue rep;
//        send_command(s,req,rep);
//    } else if (cmd == "cdpeek") {
//        if (argc != 6) {
//            printf("Usage: cdpeek femb_idx cd_idx chip_addr reg_page reg_addr\n");
//            printf("   femb_idx and cd_idx are base 10, remaining args are base 16\n");
//            return 0;
//        }
//        wib::CDPeek req;
//        req.set_femb_idx((uint8_t)strtoull(argv[1],NULL,10));
//        req.set_coldata_idx((uint8_t)strtoull(argv[2],NULL,10));
//        req.set_chip_addr((uint8_t)strtoull(argv[3],NULL,16));
//        req.set_reg_page((uint8_t)strtoull(argv[4],NULL,16));
//        req.set_reg_addr((uint8_t)strtoull(argv[5],NULL,16));
//        wib::CDRegValue rep;
//        send_command(s,req,rep);
//        printf("0x%X\n",rep.data());
//    } else if (cmd == "cdpoke") {
//        if (argc != 7) {
//            printf("Usage: cdpoke femb_idx cd_idx chip_addr reg_page reg_addr data\n");
//            printf("   femb_idx and cd_idx are base 10, remaining args are base 16\n");
//            return 0;
//        }
//        wib::CDPoke req;
//        req.set_femb_idx((uint8_t)strtoull(argv[1],NULL,10));
//        req.set_coldata_idx((uint8_t)strtoull(argv[2],NULL,10));
//        req.set_chip_addr((uint8_t)strtoull(argv[3],NULL,16));
//        req.set_reg_page((uint8_t)strtoull(argv[4],NULL,16));
//        req.set_reg_addr((uint8_t)strtoull(argv[5],NULL,16));
//        req.set_data((uint8_t)strtoull(argv[6],NULL,16));
//        wib::CDRegValue rep;
//        send_command(s,req,rep);
//    } else if (cmd == "cdfastcmd") {
//        if (argc != 2) {
//            printf("Usage: cdfastcmd cmd\n");
//            printf("   cmd can be: reset, act, sync, edge, idle, edge_act\n");
//            return 0;
//        }
//        wib::CDFastCmd req;
//        string cmd(argv[1]);
//        if (cmd == "reset") {
//            req.set_cmd(FAST_CMD_RESET);
//        } else if (cmd == "act") {
//            req.set_cmd(FAST_CMD_ACT);
//        } else if (cmd == "sync") {
//            req.set_cmd(FAST_CMD_SYNC);
//        } else if (cmd == "edge") {
//            req.set_cmd(FAST_CMD_EDGE);
//        } else if (cmd == "idle") {
//            req.set_cmd(FAST_CMD_IDLE);
//        } else if (cmd == "edge_act") {
//            req.set_cmd(FAST_CMD_EDGE_ACT);
//        } else {
//            printf("Unknown fast command: %s\n",argv[1]);
//            printf("Valid fast commands: reset, act, sync, edge, idle, edge_act\n");
//            return 0;
//        }
//        wib::Empty rep;
//        send_command(s,req,rep);
    } else if (cmd == "help") {
        print_help();
    } else {
        printf("Unrecognized Command: %s\n",argv[0]);
        return 0;
    }
    return 0;
}

int main(int argc, char **argv) {

    char *ip = (char*)"127.0.0.1";

    signed char opt;
    while ((opt = getopt(argc, argv, "w:h")) != -1) {
       switch (opt) {
           case 'h':
               print_usage(argv[0]);
               print_help();
               return 1;
           case 'w':
               ip = optarg;
               break;
           default: /* '?' */
               print_usage(argv[0]);
               return 1;
       }
    }

    zmq::context_t context(1);
    zmq::socket_t socket(context, ZMQ_REQ);

    char *addr;
    int len = asprintf(&addr,"tcp://%s:1234",ip);
    if (len < 0) return 1;

    socket.connect(addr);
    free(addr);

    if (optind < argc) {
        return run_command(socket,argc-optind,argv+optind);
    } else {
        char* buf;
        while ((buf = readline(">> ")) != nullptr) {
            if (strlen(buf) > 0) {
                add_history(buf);
            } else {
                free(buf);
                continue;
            }
            char *delim = (char*)" ";
            int count = 1;
            char *ptr = buf;
            while((ptr = strchr(ptr, delim[0])) != NULL) {
                count++;
                ptr++;
            }
            if (count > 0) {
                char **cmd = new char*[count];
                cmd[0] = strtok(buf, delim);
                int i;
                for (i = 1; cmd[i-1] != NULL && i < count; i++) {
                    cmd[i] = strtok(NULL, delim);
                }
                if (cmd[i-1] == NULL) i--;
                int ret = run_command(socket,i,cmd);
                delete [] cmd;
                if (ret == 255) return 0;
                if (ret != 0) return ret;
            } else {
                return 0;
            }
            free(buf);
        }
    }

    return 0;
}
