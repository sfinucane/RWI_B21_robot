#!/usr/bin/perl



# Send the button press to zazaconsole
#sub sendit {
   my ($button) = $ARGV[0];
   if ($button =~ m/blue/) {
     print "Sending blue button.\n";
   }
   elsif ($button =~ m/green/) {
     print "Sending green button.\n";
   }
   elsif ($button =~ m/red/) {
     print "Sending red button.\n";
   }
   else {
     die "$button button not supported!";
   }

  # system("lynx -dump -error_file=/home/httpd/html/logs/sendbutton-errors.log http://zazaconsole/cgi-bin/posServer?button_press=$button >/home/httpd/html/logs/sendbutton.log");
   system("wget -o /home/httpd/html/logs/sendbutton.log http://zazaconsole/cgi-bin/posServer?button_press=$button ");

#}

