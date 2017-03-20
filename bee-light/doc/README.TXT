BeeSoft API library Readme.txt (May 17, 1999)

Robot Learning Laboratory
Carnegie Mellon University


*** Preface ***

Re-implementation of BeeSoft Light is one of the student projects of course 
15-822, System Design & Implementation Practicum, offered by Computer Science 
Department, at Carnegie Mellon University. The goal of this course is to 
provide a hands-on learning opportunity to acquire skills relevant to the 
design, implementation, evaluation and tuning of complex systems. This course 
is project-based, with each student doing a different project. The projects 
are "real" rather than contrived. Every project addresses the needs of some 
"customers", which may be CS or Robotics research projects. Sebastian 
Thrun, assistant professor of Computer Science Department at CMU, sponsors 
"Re-implementation of BeeSoft Light" project. This project is to re-design 
and re-implement BeeSoft Light API library. Then his Robot Learning Laboratory 
can write programs to navigate their robots more easily than before.

*** List Of Files ***


The BeeSoft package consists of the following files.
  
Document:
Readme.txt
CS15-822Description.txt
ProjectReport.doc
ProjectReport.html
ProjectReport.tex
ProjectReport.ps
BeeSoftAPIManual.doc
BeeSoftAPIManual.html
BeeSoftAPIManual.tex
BeeSoftAPIManual.ps
Unix Manual pages
./man/man3/*.3

Source files:
./src/bee.c
Library files
./lib/libbee.a
./lib/libtcx.a
/usr/lib/libpthread.a (not included in the tar)
Example files
./example
Data files
./data/localize/localize

Other important files not included in the tar:
BeeSoft path = ~/bee  or /project1/mercator/software/external/beeSoft/bee 
BeeSoft path/bin/tcxServer
BeeSoft path/bin/baseServer or BeeSoft path/pioneer
BeeSoft path/bin/laserServer
BeeSoft path/bin/colliServer
BeeSoft path/bin/simulator
BeeSoft path/bin/LOCALIZE
BeeSoft path/bin/plan
BeeSoft path/bin/DISPLAY_LOCALIZE or
BeeSoft path/bin/SET_ROBOT
BeeSoft path/etc/beeSoft.ini

*** Notes ***
1. How to read Manual pages:
   "setenv MANPATH" first. For example, run "setenv MANPATH BeeSoftAPI/man".
   Then you could run "man beeInitialize" to view the pages. Those files
   are placed in man3 sub-directory according to some suggested rules 
   (man3 is for library documents).
   
2. How to edit the Manual pages:
   It's nroff/troff/groff format. I did not read any guide book for the 
   nroff processing directives used in the manual pages.
   You could try to read those files in text editor (like vi or emacs) and 
   the instruction in note 1 to understand the usage of those nroff processing 
   directives.

3. How to program
   Take a look at source code in example subdirectory. Write your own program.
   Add an entry in Makefile file. Then run "make"

4. How to modify the BeeSoftAPI library.
   Modify bee.c in src subdirectory.
   Modify bee.h in example subdirectory.
   Run "make bee.o" in src subdirectory.
   Run "ar cru libbee.a bee.o" and then "mv libbee.a ../lib". But you also could
   run "make libbee.a".

*** Running examples ***

The best way to understand the package is to run example programs first.
You could run them in simulator program. This section tells you about the
procedure to run example programs.

user.c (user as its executive) lists all the movement commands. You could
run it to know the functions of the movement commands. 

To run local movement commands, run the following server modules first.
The file specified in command parameters are in data/localize/localize  .
(Notice: each server module must be running in different shell. You might need
         a X-windows workstation. In each shell you should set an environment
         variable: setenv TCXHOST xxxx
         xxxx is the machine name which you run tcxServer. 
         You could also write it in a .beeSoft or .cshrc file in your home 
         directory   )
          
1. >tcxServer
2. >simulator -map floor.sim
   Then a window called "simulator" appears. (The window whose titlebar shows 
   "simulator")
3. >colliServer -noindex +simulator +laser
   Then the "simulator" window shows a map and a red robot point.
   "colliServer -help" will show the meanings of the parameters.
   Now you could have fun in simulator window now. Clicking in
   the map will change the robot's position. Middle-click in
   the map will give the robot a destination to move. Try to
   click the buttons on the right-handed side to know their
   functions.

Then you could run "user" now. But you could only use commands 1-12 and
14, 15, 18 because they are local movement commands.

To run command 13 "beeApproachAbsolute()", you need to run "LOCALIZE"
server module.

4. >SET_ROBOT -gmap floor.map
5. >LOCALIZE laser.ini
   Type enter in the shell which runs LOCALIZE.
   Then two windows "Samples" and "Robot window" will appear. 
   In addition, window "Map" and a small window "Place Robot" should appear 
   at the same time. First you should set a initial position for the robot. 
   You do so by clicking at a point in the "Map" window. Then you click DONE 
   button in the "Place Robot" window. You will see the robot change its 
   position to the point you specified in "Simulator", "Robot window", and 
   "Samples".

Now re-run "user". Now you could run command 13.

To run command 16 "beeGotoRelative()" and 17 "beeGotoAbsolute()", you need
to run "Planner" module in addition".

6. >plan
   You need to run it before LOCALIZE starts to run.
   Then a "BeeSoft Planner" window appears. Planner server module compute
   the optimal route for the robot to reach its destination.
   In this window you could click a point in the map and then click 
   "autonomous ON" button to ask the robot to move too. 

*** Comment ***

You will have fun with Simulator. But sometimes your robot will have strange
behaviors in Simulator. Then you might need to run the program in real robots. 
That is not all your fault. (And not my bugs either :-) )

*** Contact ***

Contact:
Jianjung (James) Ying
jying@mail.com


