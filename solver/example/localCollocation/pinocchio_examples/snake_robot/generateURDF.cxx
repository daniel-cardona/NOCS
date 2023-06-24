/*
 *
 * Copyright (C) 2020
 * Daniel Cardona-Ortiz <daniel.cardona@cinvestav.edu.mx>, Gustavo Arechavaleta <garechav@cinvestav.edu.mx>
 * CINVESTAV - Saltillo Campus
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */


#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <string>


void createURDF(int N) {

    char fileName[50];

    int n=sprintf(fileName,"./urdf/snake_robot_%02d.urdf",N);

    std::ofstream outfile(fileName);
    std::ofstream datafile("./urdf/0_dataFile.txt",std::ios::app);

    datafile<< N <<std::endl;

    // Write the header
    outfile << "<?xml version=\"1.0\"?>\n";
    outfile << "<robot name=\"snake_robot\">\n";

    // Write the base link
    outfile << "  <link name=\"ROOT\">\n";
    outfile << "    <visual>\n";
    outfile << "      <geometry>\n";
    outfile << "        <cylinder length=\"0.6\" radius=\"0.2\" />\n";
    outfile << "      </geometry>\n";
    outfile<<  "      <origin rpy=\"0 0 0\" xyz=\"0 0 0\" />\n ";
    outfile << "    </visual>\n";
    outfile << "    <collision>\n";
    outfile << "      <geometry>\n";
    outfile << "       <cylinder length=\"0.6\" radius=\"0.2\" />\n";
    outfile << "      </geometry>\n";
    outfile << "    </collision>\n";
    outfile << "    <inertial>\n";
    outfile << "      <mass value=\"0.1\" />\n";
    outfile << "      <origin rpy=\"0 0 0\" xyz=\"0 0 0\" />\n";
    outfile << "      <inertia ixx=\"0.4\" ixy=\"0.0\" ixz=\"0.0\" iyy=\"0.4\" iyz=\"0.0\" izz=\"0.2\" />\n";
    outfile << "    </inertial>\n";
    outfile << "  </link>\n";

    // Write the joints and links for each degree of freedom
    for (int i = 0; i < N; i++) {

        int link_id=i+1;
        int joint_id=i+1;
        std::string linkName = "link" + std::to_string(link_id);
        std::string jointName = "joint" + std::to_string(joint_id);

        outfile << "  <link name=\"" << linkName << "\">\n";
        outfile << "    <visual>\n";
        outfile << "      <geometry>\n";
        outfile << "        <cylinder radius=\"0.2\" length=\"0.6\" />\n";
        outfile << "      </geometry>\n";
        outfile<<  "      <origin rpy=\"0 0 0\" xyz=\"0 0 0.3\" />\n ";
        outfile << "    </visual>\n";
        outfile << "    <collision>\n";
        outfile << "      <geometry>\n";
        outfile << "        <cylinder radius=\"0.2\" length=\"0.6\" />\n";
        outfile << "      </geometry>\n";
        outfile << "    </collision>\n";
        outfile << "    <inertial>\n";
        outfile << "      <mass value=\"0.1\" />\n";
        //outfile << "      <origin rpy=\"0 0 0\" xyz=\"0 0 0.05\" />\n";
        outfile << "      <inertia ixx=\"0.4\" ixy=\"0.0\" ixz=\"0.0\" iyy=\"0.4\" iyz=\"0.0\" izz=\"0.2\" />\n";
        outfile << "    </inertial>\n";
        outfile << "  </link>\n";

        if(joint_id==1){

            outfile << "  <joint name=\"" << jointName << "\" type=\"revolute\">\n";
            outfile << "    <parent link=\"ROOT\" />\n";
            outfile << "    <child link=\"" << linkName << "\" />\n";
            outfile << "    <origin rpy=\"0 0 0\" xyz=\"0 0 0.3\" />\n";
            outfile << "    <axis xyz=\"0 1 0\" />\n";
            outfile << "    <limit effort=\"1000\" lower=\"-1.0\" upper=\"1.0\" velocity=\"1.0\" />\n";
            outfile << "  </joint>\n";
            continue;
        }//The first joint is different

        outfile << "  <joint name=\"" << jointName << "\" type=\"revolute\">\n";
        outfile << "    <parent link=\"link" << i << "\" />\n";
        outfile << "    <child link=\"" << linkName << "\" />\n";
        outfile << "    <origin rpy=\"0 0 0\" xyz=\"0 0 0.6\" />\n";

        if(joint_id%2==0){
            outfile << "    <axis xyz=\"1 0 0\" />\n";
        }else{
            outfile << "    <axis xyz=\"0 1 0\" />\n";
        }
        outfile << "    <limit effort=\"1000\" lower=\"-1\" upper=\"1\" velocity=\"1\" />\n";
        outfile << "  </joint>\n";
    }

    // Write the closing tag
    outfile << "</robot>\n";


    datafile.close();
    outfile.close();
}

int main(int argc,char* argv[])
{

    int maxNumberURDF=33;
    int dofInterval=3;

    int nDoF=dofInterval;

    for(int i=0; i<maxNumberURDF; i++){

        createURDF(nDoF);
        nDoF+=dofInterval;

    }




    return(0);
}
