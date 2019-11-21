# Changelog for Manipulative Arm
## Matt Haberbusch and Rahul Pokharna


### Nov 21, 2019
Tested more cutting parameters, to see how well cutting works through layers of mylar, and created a document to record parameters and descriptions of results. More refining to come, work done for CDR. 

### Nov 20, 2019
Cleaned up PTEL\_y and PTEL\_z to match changes made in PTEL\_x. Planned out and began implementation for determining task plane skills and components, on how to find a plane from 3 points. Changed what set\_task\_frame service callback function (in accomodation controller) does in accomodation controller, for it to use 3 points set with a new service call (in progress).

### Nov 19, 2019
Created skill to pull in the direction of the blade, but keeping the blade tip in the plane of the task. It needs to eventually be separated/made more clear on functionality. Cleaned up the PTEL movements to use less if statements and have clearer use of variables and math. We modified parameters so we were able to cut mylar more effectively, and found the appropriate length of the end effector with the knife. 

### Nov 18, 2019
We added more parameters so we have a tool and task frame option for each skill. We also changed how we got the values for the threshold of the orientation, and updated what we printed out. We also changed parameters to make the process of drawing the boxes and turning in the task frame easier and functional. 

### Nov 14, 2019
We did more work toward orientation targeting in the task frame, trying out new rotation math to get the rotations around the correct axis. 
