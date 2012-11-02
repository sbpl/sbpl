/*
 * Copyright (c) 2008, Maxim Likhachev
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Pennsylvania nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <iostream>
#include <string>
#include <fstream>
#include <gtest/gtest.h>

using namespace std;

#include "../headers.h"

static const std::string PATH_PREFIX("src/test/");

void diffTest(const std::string& outputStr)
{
    std::string validOutputStr(outputStr + ".valid");
    std::ifstream validFile(validOutputStr.c_str());
    // If there is no valid file then generate one.
    if (!validFile.good()) {
        std::ifstream newOutputFile(outputStr.c_str());
        std::stringbuf sbuf;
        newOutputFile >> &sbuf;
        std::ofstream newValidFile(validOutputStr.c_str());
        newValidFile << sbuf.str();
    }
    else { // Verify output against the valid file.
        std::stringbuf newStrBuf, validStrBuf;
        std::ifstream fNew(outputStr.c_str()), fValid(validOutputStr.c_str());
        fNew >> &newStrBuf;
        fValid >> &validStrBuf;
        ASSERT_EQ(newStrBuf.str() == validStrBuf.str(), true);
    }
}

void runARAPlannerTest(const std::string& problem)
{
    try {
        double allocated_time_secs = 0.5; // in seconds
        MDPConfig MDPCfg;
        EnvironmentNAV2D environment_nav2D;
        std::string problemStr = PATH_PREFIX + problem;
        ASSERT_EQ(environment_nav2D.InitializeEnv(problemStr.c_str()), true);
        ASSERT_EQ(environment_nav2D.InitializeMDPCfg(&MDPCfg), true);

        // plan a path
        vector<int> solution_stateIDs_V;
        ARAPlanner ara_planner(&environment_nav2D, false);
        ASSERT_EQ(ara_planner.set_start(MDPCfg.startstateid), true);
        ASSERT_EQ(ara_planner.set_goal(MDPCfg.goalstateid), true);
        ASSERT_EQ(ara_planner.replan(allocated_time_secs, &solution_stateIDs_V), true);

        // output the path
        std::string outputStr = problemStr + ".out";
        FILE* fSol = SBPL_FOPEN(outputStr.c_str(), "w");
        for (unsigned int i = 0; i < solution_stateIDs_V.size(); i++) {
            environment_nav2D.PrintState(solution_stateIDs_V[i], true, fSol);
        }

        SBPL_FCLOSE(fSol);

        // Now apply the file diff test
        diffTest(outputStr);
    }
    catch (...) {
        FAIL() << "Uncaught exception : " << "This is OK on OS X";
    }
}

TEST(araplanner, env1)
{
    runARAPlannerTest("env1.cfg");
}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

