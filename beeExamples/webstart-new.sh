#!/bin/sh
#

#sleep 10

# Startup Phase II mode
#/home/bee/speakit2.pl Zaza startup initiated.
lynx -dump -error_file=/home/httpd/html/logs/startup-errors.log http://localhost/cgi-bin/zaza-control-simple?op_mode=Phase2 >/home/httpd/html/logs/startup.log


# We need to shutdown, and start again for reliable auto-startup with the new Beesoft
sleep 20
#/home/bee/speakit2.pl Please standby, I am restarting.
sleep 10
lynx -dump -error_file=/home/httpd/html/logs/startup-errors.log http://localhost/cgi-bin/zaza-control-simple?op_mode=ShutDown >/home/httpd/html/logs/startup.log
sleep 10
lynx -dump -error_file=/home/httpd/html/logs/startup-errors.log http://localhost/cgi-bin/zaza-control-simple?op_mode=Phase2 >/home/httpd/html/logs/startup.log

