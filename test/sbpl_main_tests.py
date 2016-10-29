#!/usr/bin/env python

from __future__ import print_function

from sys import argv, exit
from subprocess import call, check_call, STDOUT
from os import getcwd, chdir, pardir, devnull, mkdir
from os.path import join, exists, abspath, basename
import time

colors = {
    'grey': 30,         'red': 31,
    'green': 32,        'yellow': 33,
    'blue': 34,         'magenta': 35,
    'cyan': 36,         'white': 37,

    'lightgrey' : 90,   'lightred' : 91,
    'lightgreen' : 92,  'lightyellow' : 93,
    'lightblue' : 94,   'lighmagneta' : 95,
    'lightcyan' : 96,   'lighwhite' : 97,
}

def __print(*args, **kwargs):
    print(*args, **kwargs)

def print_status(text):
    __print("\033[" + str(colors['lightblue']) + "m" + "[i]" + "\033[0m" + " " + text)

def print_warning(text):
    __print("\033[" + str(colors['yellow']) + "m" + "[!]" + "\033[0m" + " " + text)

def print_fail(text):
    __print("\033[" + str(colors['lightred']) + "m" + "[-]" + "\033[0m" + " " + text)

def print_success(text):
    __print("\033[1;" + str(colors['lightgreen']) + "m" + "[+]" + " " + text + "\033[0m")

def print_summary(text):
    __print("\033[1;" + str(colors['lightcyan']) + "m" + text + "\033[0m" )

def print_blue(text):
    __print("\033[" + str(colors['blue']) + "m" + text + "\033[0m" )

def print_green(text):
    __print("\033[" + str(colors['green']) + "m" + text + "\033[0m" )

def print_red(text):
    __print("\033[" + str(colors['red']) + "m" + text + "\033[0m" )


def generate_makefile(build_path='', source_path=''):
    """
    Generates a Makefile for SBPL if one doesn't exist

    Looks in a directory relative to the current directory for a Makefile. A new one is generated
    using CMake if one isn't already there. CMakeLists.txt must exist in that directory for this to
    work.

    @return Whether or not a Makefile was generated
    """

    print_status('Looking for Makefile in {}'.format(build_path))
    if not exists(join(build_path, 'Makefile')):
        if not exists(join(source_path, 'CMakeLists.txt')):
            return 1
        else:
            print_warning('No Makefile found for SBPL, running cmake')

            cmake_args = []
            cmake_args.append('-DCMAKE_RUNTIME_OUTPUT_DIRECTORY={}'.format(build_path + '/bin'))
            cmake_args.append('-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={}'.format(build_path + '/lib'))

            cmd = [
                "cmake",
                source_path,
            ]

            cmd += cmake_args

            configure_code = check_call(
                ' '.join(cmd),
                shell=True,
                stderr=STDOUT,
                cwd=build_path)

            if configure_code:
                print_fail("Invoking {} failed".format(' '.join(cmd)))

            return configure_code

    return 0

def build_project(build_path=''):
    print_status("Invoke build step")

    cmd = [
        "cmake",
        "--build",
        "."
    ]

    return check_call(
        ' '.join(cmd),
        shell=True,
        stderr=STDOUT,
        cwd=build_path
    )

def run_sbpl_test(env_type, planner_type, test_env, mprim, is_forward_search, navigating=False, sbpl_root='', sbpl_exe='test_sbpl'):
    """
    @brief run the sbpl test executable
    """
    devnull_fd = open(devnull) # for surpressing output

    test_env_path = join(sbpl_root, test_env)
    mprim_path = join(sbpl_root, mprim)

    print_green("=" * 13)
    print_status("Running {planner_type} planner on {env_type} environment".format(planner_type=planner_type, env_type=env_type))
    print_status("Navigating = {navigating}".format(navigating=navigating))
    print_status("Test environment = {test_env}".format(test_env=test_env))
    print_status("Motion primitives = {mprim}".format(mprim=mprim))

    forward_search_arg = ''
    if is_forward_search:
        forward_search_arg = 'forward'
    else:
        forward_search_arg = 'backward'

    args = [sbpl_exe, '--env=' + env_type, '--planner=' + planner_type, '--search-dir=' + forward_search_arg, test_env_path, mprim_path]
    if mprim == '': args.pop()
    if navigating: args.insert(1, '-s')
    for arg in args:
        print(arg)

    start_time = time.time()

    sbpl_res = call(args, stdout=devnull_fd, stderr=devnull_fd)

    end_time = time.time()

    print("Planning took {} seconds.".format(end_time - start_time))

    if sbpl_res == 0:
        print_success("Planning succeeded.")
    else:
        print_fail("Planner failed with exit code {}.".format(sbpl_res))

    devnull_fd.close()

    return sbpl_res
#end run_sbpl_test

def test_forward_serach_2d_environments(sbpl_root, sbpl_exe):
    print_blue("#" * 33)
    print_status("### PLANNING FOR 2D ENVIRONMENTS ###")

    num_2d_test_successes = 0

    # all planners on 2d environment (12 tests) env1
    if run_sbpl_test('2d', 'arastar', 'env_examples/nav2d/env1.cfg', 'matlab/mprim/pr2.mprim', True, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_2d_test_successes = num_2d_test_successes + 1
    if run_sbpl_test('2d', 'adstar', 'env_examples/nav2d/env1.cfg', 'matlab/mprim/pr2.mprim', True, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_2d_test_successes = num_2d_test_successes + 1
    if run_sbpl_test('2d', 'anastar', 'env_examples/nav2d/env1.cfg', 'matlab/mprim/pr2.mprim', True, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_2d_test_successes = num_2d_test_successes + 1
    if run_sbpl_test('2d', 'rstar', 'env_examples/nav2d/env1.cfg', 'matlab/mprim/pr2.mprim', True, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_2d_test_successes = num_2d_test_successes + 1

    # all planners navigating on 2d env1
    if run_sbpl_test('2d', 'arastar', 'env_examples/nav2d/env1.cfg', 'matlab/mprim/pr2.mprim', True, True, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_2d_test_successes = num_2d_test_successes + 1
    if run_sbpl_test('2d', 'adstar', 'env_examples/nav2d/env1.cfg', 'matlab/mprim/pr2.mprim', True, True, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_2d_test_successes = num_2d_test_successes + 1
    if run_sbpl_test('2d', 'anastar', 'env_examples/nav2d/env1.cfg', 'matlab/mprim/pr2.mprim', True, True, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_2d_test_successes = num_2d_test_successes + 1
    if run_sbpl_test('2d', 'rstar', 'env_examples/nav2d/env1.cfg', 'matlab/mprim/pr2.mprim', True, True, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_2d_test_successes = num_2d_test_successes + 1

    # all planners on 2d env2
    if run_sbpl_test('2d', 'arastar', 'env_examples/nav2d/env2.cfg', 'matlab/mprim/pr2.mprim', True, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_2d_test_successes = num_2d_test_successes + 1
    if run_sbpl_test('2d', 'adstar', 'env_examples/nav2d/env2.cfg', 'matlab/mprim/pr2.mprim', True, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_2d_test_successes = num_2d_test_successes + 1
    if run_sbpl_test('2d', 'anastar', 'env_examples/nav2d/env2.cfg', 'matlab/mprim/pr2.mprim', True, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_2d_test_successes = num_2d_test_successes + 1
    if run_sbpl_test('2d', 'rstar', 'env_examples/nav2d/env2.cfg', 'matlab/mprim/pr2.mprim', True, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_2d_test_successes = num_2d_test_successes + 1

    # all planners navigating on 2d env2 (no thanks, I want my tests to finish)
    #run_sbpl_test('2d', 'arastar', 'env_examples/nav2d/env2.cfg', 'matlab/mprim/pr2.mprim', True)
    #run_sbpl_test('2d', 'adstar', 'env_examples/nav2d/env2.cfg', 'matlab/mprim/pr2.mprim', True)
    #run_sbpl_test('2d', 'anastar', 'env_examples/nav2d/env2.cfg', 'matlab/mprim/pr2.mprim', True)
    #run_sbpl_test('2d', 'rstar', 'env_examples/nav2d/env2.cfg', 'matlab/mprim/pr2.mprim', True)

    return num_2d_test_successes

def test_forward_serach_xytheta_environments(sbpl_root, sbpl_exe):
    print_blue("#" * 33)
    print_status("### PLANNING FOR (X,Y,THETA) ENVIRONMENTS (9 tests) ###")

    num_xytheta_test_successes = 0

    # all planners on xytheta env1
    if run_sbpl_test('xytheta', 'arastar', 'env_examples/nav3d/env1.cfg', 'matlab/mprim/pr2.mprim', True, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_xytheta_test_successes = num_xytheta_test_successes + 1
    if run_sbpl_test('xytheta', 'adstar', 'env_examples/nav3d/env1.cfg', 'matlab/mprim/pr2.mprim', True, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_xytheta_test_successes = num_xytheta_test_successes + 1
    if run_sbpl_test('xytheta', 'anastar', 'env_examples/nav3d/env1.cfg', 'matlab/mprim/pr2.mprim', True, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_xytheta_test_successes = num_xytheta_test_successes + 1

    # all planners navigating on xytheta env1
    if run_sbpl_test('xytheta', 'arastar', 'env_examples/nav3d/env1.cfg', 'matlab/mprim/pr2.mprim', True, True, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_xytheta_test_successes = num_xytheta_test_successes + 1
    if run_sbpl_test('xytheta', 'adstar', 'env_examples/nav3d/env1.cfg', 'matlab/mprim/pr2.mprim', True, True, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_xytheta_test_successes = num_xytheta_test_successes + 1
    if run_sbpl_test('xytheta', 'anastar', 'env_examples/nav3d/env1.cfg', 'matlab/mprim/pr2.mprim', True, True, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_xytheta_test_successes = num_xytheta_test_successes + 1

    # all planners on xytheta env2
    if run_sbpl_test('xytheta', 'arastar', 'env_examples/nav3d/env2.cfg', 'matlab/mprim/pr2_10cm.mprim', True, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_xytheta_test_successes = num_xytheta_test_successes + 1
    if run_sbpl_test('xytheta', 'adstar', 'env_examples/nav3d/env2.cfg', 'matlab/mprim/pr2_10cm.mprim', True, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_xytheta_test_successes = num_xytheta_test_successes + 1
    if run_sbpl_test('xytheta', 'anastar', 'env_examples/nav3d/env2.cfg', 'matlab/mprim/pr2_10cm.mprim', True, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_xytheta_test_successes = num_xytheta_test_successes + 1

    # all planners navigating on xytheta env2 (no thanks, i want my tests to finish)
    #run_sbpl_test('xytheta', 'arastar', 'env_examples/nav3d/env2.cfg', 'matlab/mprim/pr2_10cm.mprim', True)
    #run_sbpl_test('xytheta', 'adstar', 'env_examples/nav3d/env2.cfg', 'matlab/mprim/pr2_10cm.mprim', True)
    #run_sbpl_test('xytheta', 'anstar', 'env_examples/nav3d/env2.cfg', 'matlab/mprim/pr2_10cm.mprim', True)

    return num_xytheta_test_successes

def test_forward_serach_xythetamlev_environments(sbpl_root, sbpl_exe):
    print_blue("#" * 33)
    print_status("### PLANNING FOR (X,Y,THETA,LEV) ENVIRONMENTS (6 tests) ###")

    num_xythetamlev_test_successes = 0

    # all planners on xythetamlev env1
    if run_sbpl_test('xythetamlev', 'arastar', 'env_examples/nav3d/env1.cfg', 'matlab/mprim/pr2.mprim', True, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_xythetamlev_test_successes = num_xythetamlev_test_successes + 1
    if run_sbpl_test('xythetamlev', 'adstar', 'env_examples/nav3d/env1.cfg', 'matlab/mprim/pr2.mprim', True, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_xythetamlev_test_successes = num_xythetamlev_test_successes + 1
    if run_sbpl_test('xythetamlev', 'anastar', 'env_examples/nav3d/env1.cfg', 'matlab/mprim/pr2.mprim', True, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_xythetamlev_test_successes = num_xythetamlev_test_successes + 1

    # all planners on xythetamlev env2
    if run_sbpl_test('xythetamlev', 'arastar', 'env_examples/nav3d/env2.cfg', 'matlab/mprim/pr2_10cm.mprim', True, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_xythetamlev_test_successes = num_xythetamlev_test_successes + 1
    if run_sbpl_test('xythetamlev', 'adstar', 'env_examples/nav3d/env2.cfg', 'matlab/mprim/pr2_10cm.mprim', True, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_xythetamlev_test_successes = num_xythetamlev_test_successes + 1
    if run_sbpl_test('xythetamlev', 'anastar', 'env_examples/nav3d/env2.cfg', 'matlab/mprim/pr2_10cm.mprim', True, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_xythetamlev_test_successes = num_xythetamlev_test_successes + 1

    return num_xythetamlev_test_successes

def test_forward_serach_robarm_environments(sbpl_root, sbpl_exe):
    print_blue("#" * 33)
    print_status("### PLANNING FOR ROBARM ENVIRONMENTS (12 tests) ###")

    num_robarm_test_successes = 0

    # all planners on robarm env1
    if run_sbpl_test('robarm', 'arastar', 'env_examples/robarm/env1_6d.cfg', '', True, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_robarm_test_successes = num_robarm_test_successes + 1
    if run_sbpl_test('robarm', 'adstar', 'env_examples/robarm/env1_6d.cfg', '', True, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_robarm_test_successes = num_robarm_test_successes + 1
    if run_sbpl_test('robarm', 'anastar', 'env_examples/robarm/env1_6d.cfg', '', True, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_robarm_test_successes = num_robarm_test_successes + 1
    if run_sbpl_test('robarm', 'rstar', 'env_examples/robarm/env1_6d.cfg', '', True, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_robarm_test_successes = num_robarm_test_successes + 1

    # all planners on robarm env2
    if run_sbpl_test('robarm', 'arastar', 'env_examples/robarm/env2_6d.cfg', '', True, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_robarm_test_successes = num_robarm_test_successes + 1
    if run_sbpl_test('robarm', 'adstar', 'env_examples/robarm/env2_6d.cfg', '', True, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_robarm_test_successes = num_robarm_test_successes + 1
    if run_sbpl_test('robarm', 'anastar', 'env_examples/robarm/env2_6d.cfg', '', True, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_robarm_test_successes = num_robarm_test_successes + 1
    if run_sbpl_test('robarm', 'rstar', 'env_examples/robarm/env2_6d.cfg', '', True, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_robarm_test_successes = num_robarm_test_successes + 1

    # all planners on robarm env3
    if run_sbpl_test('robarm', 'arastar', 'env_examples/robarm/env3_6d.cfg', '', True, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_robarm_test_successes = num_robarm_test_successes + 1
    if run_sbpl_test('robarm', 'adstar', 'env_examples/robarm/env3_6d.cfg', '', True, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_robarm_test_successes = num_robarm_test_successes + 1
    if run_sbpl_test('robarm', 'anastar', 'env_examples/robarm/env3_6d.cfg', '', True, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_robarm_test_successes = num_robarm_test_successes + 1
    if run_sbpl_test('robarm', 'rstar', 'env_examples/robarm/env3_6d.cfg', '', True, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_robarm_test_successes = num_robarm_test_successes + 1

    return num_robarm_test_successes

def test_backward_serach_2d_environments(sbpl_root, sbpl_exe):
    print_blue("#" * 33)
    print_status("### PLANNING FOR 2D ENVIRONMENTS ###")

    num_b_2d_test_successes = 0

    # all planners on 2d environment (12 tests) env1
    if run_sbpl_test('2d', 'arastar', 'env_examples/nav2d/env1.cfg', 'matlab/mprim/pr2.mprim', False, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_b_2d_test_successes = num_b_2d_test_successes + 1
    if run_sbpl_test('2d', 'adstar', 'env_examples/nav2d/env1.cfg', 'matlab/mprim/pr2.mprim', False, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_b_2d_test_successes = num_b_2d_test_successes + 1
    if run_sbpl_test('2d', 'anastar', 'env_examples/nav2d/env1.cfg', 'matlab/mprim/pr2.mprim', False, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_b_2d_test_successes = num_b_2d_test_successes + 1
    if run_sbpl_test('2d', 'rstar', 'env_examples/nav2d/env1.cfg', 'matlab/mprim/pr2.mprim', False, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_b_2d_test_successes = num_b_2d_test_successes + 1

    # all planners navigating on 2d env1
    if run_sbpl_test('2d', 'arastar', 'env_examples/nav2d/env1.cfg', 'matlab/mprim/pr2.mprim', False, True, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_b_2d_test_successes = num_b_2d_test_successes + 1
    if run_sbpl_test('2d', 'adstar', 'env_examples/nav2d/env1.cfg', 'matlab/mprim/pr2.mprim', False, True, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_b_2d_test_successes = num_b_2d_test_successes + 1
    if run_sbpl_test('2d', 'anastar', 'env_examples/nav2d/env1.cfg', 'matlab/mprim/pr2.mprim', False, True, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_b_2d_test_successes = num_b_2d_test_successes + 1
    if run_sbpl_test('2d', 'rstar', 'env_examples/nav2d/env1.cfg', 'matlab/mprim/pr2.mprim', False, True, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_b_2d_test_successes = num_b_2d_test_successes + 1

    # all planners on 2d env2
    if run_sbpl_test('2d', 'arastar', 'env_examples/nav2d/env2.cfg', 'matlab/mprim/pr2.mprim', False, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_b_2d_test_successes = num_b_2d_test_successes + 1
    if run_sbpl_test('2d', 'adstar', 'env_examples/nav2d/env2.cfg', 'matlab/mprim/pr2.mprim', False, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_b_2d_test_successes = num_b_2d_test_successes + 1
    if run_sbpl_test('2d', 'anastar', 'env_examples/nav2d/env2.cfg', 'matlab/mprim/pr2.mprim', False, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_b_2d_test_successes = num_b_2d_test_successes + 1
    if run_sbpl_test('2d', 'rstar', 'env_examples/nav2d/env2.cfg', 'matlab/mprim/pr2.mprim', False, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_b_2d_test_successes = num_b_2d_test_successes + 1

    # all planners navigating on 2d env2 (no thanks, I want my tests to finish)
    #run_sbpl_test('2d', 'arastar', 'env_examples/nav2d/env2.cfg', 'matlab/mprim/pr2.mprim', False)
    #run_sbpl_test('2d', 'adstar', 'env_examples/nav2d/env2.cfg', 'matlab/mprim/pr2.mprim', False)
    #run_sbpl_test('2d', 'anastar', 'env_examples/nav2d/env2.cfg', 'matlab/mprim/pr2.mprim', False)
    #run_sbpl_test('2d', 'rstar', 'env_examples/nav2d/env2.cfg', 'matlab/mprim/pr2.mprim', False)

    return num_b_2d_test_successes

def test_bachward_serach_xytheta_environments(sbpl_root, sbpl_exe):
    print_blue("#" * 33)
    print_status("### PLANNING FOR (X,Y,THETA) ENVIRONMENTS (9 tests) ###")

    num_b_xytheta_test_successes = 0

    # all planners on xytheta env1
    if run_sbpl_test('xytheta', 'arastar', 'env_examples/nav3d/env1.cfg', 'matlab/mprim/pr2.mprim', False, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_b_xytheta_test_successes = num_b_xytheta_test_successes + 1
    if run_sbpl_test('xytheta', 'adstar', 'env_examples/nav3d/env1.cfg', 'matlab/mprim/pr2.mprim', False, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_b_xytheta_test_successes = num_b_xytheta_test_successes + 1
    if run_sbpl_test('xytheta', 'anastar', 'env_examples/nav3d/env1.cfg', 'matlab/mprim/pr2.mprim', False, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_b_xytheta_test_successes = num_b_xytheta_test_successes + 1

    # all planners navigating on xytheta env1
    if run_sbpl_test('xytheta', 'arastar', 'env_examples/nav3d/env1.cfg', 'matlab/mprim/pr2.mprim', False, False, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_b_xytheta_test_successes = num_b_xytheta_test_successes + 1
    if run_sbpl_test('xytheta', 'adstar', 'env_examples/nav3d/env1.cfg', 'matlab/mprim/pr2.mprim', False, False, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_b_xytheta_test_successes = num_b_xytheta_test_successes + 1
    if run_sbpl_test('xytheta', 'anastar', 'env_examples/nav3d/env1.cfg', 'matlab/mprim/pr2.mprim', False, False, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_b_xytheta_test_successes = num_b_xytheta_test_successes + 1

    # all planners on xytheta env2
    if run_sbpl_test('xytheta', 'arastar', 'env_examples/nav3d/env2.cfg', 'matlab/mprim/pr2_10cm.mprim', False, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_b_xytheta_test_successes = num_b_xytheta_test_successes + 1
    if run_sbpl_test('xytheta', 'adstar', 'env_examples/nav3d/env2.cfg', 'matlab/mprim/pr2_10cm.mprim', False, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_b_xytheta_test_successes = num_b_xytheta_test_successes + 1
    if run_sbpl_test('xytheta', 'anastar', 'env_examples/nav3d/env2.cfg', 'matlab/mprim/pr2_10cm.mprim', False, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_b_xytheta_test_successes = num_b_xytheta_test_successes + 1

    # all planners navigating on xytheta env2 (no thanks, i want my tests to finish)
    #run_sbpl_test('xytheta', 'arastar', 'env_examples/nav3d/env2.cfg', 'matlab/mprim/pr2_10cm.mprim', False)
    #run_sbpl_test('xytheta', 'adstar', 'env_examples/nav3d/env2.cfg', 'matlab/mprim/pr2_10cm.mprim', False)
    #run_sbpl_test('xytheta', 'anstar', 'env_examples/nav3d/env2.cfg', 'matlab/mprim/pr2_10cm.mprim', False)

    return num_b_xytheta_test_successes

def test_backward_serach_xythetamlev_environments(sbpl_root, sbpl_exe):
    print_blue("#" * 33)
    print_status("### PLANNING FOR (X,Y,THETA,LEV) ENVIRONMENTS (6 tests) ###")

    num_b_xythetamlev_test_successes = 0

    # all planners on xythetamlev env1
    if run_sbpl_test('xythetamlev', 'arastar', 'env_examples/nav3d/env1.cfg', 'matlab/mprim/pr2.mprim', False, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_b_xythetamlev_test_successes = num_b_xythetamlev_test_successes + 1
    if run_sbpl_test('xythetamlev', 'adstar', 'env_examples/nav3d/env1.cfg', 'matlab/mprim/pr2.mprim', False, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_b_xythetamlev_test_successes = num_b_xythetamlev_test_successes + 1
    if run_sbpl_test('xythetamlev', 'anastar', 'env_examples/nav3d/env1.cfg', 'matlab/mprim/pr2.mprim', False, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_b_xythetamlev_test_successes = num_b_xythetamlev_test_successes + 1

    # all planners on xythetamlev env2
    if run_sbpl_test('xythetamlev', 'arastar', 'env_examples/nav3d/env2.cfg', 'matlab/mprim/pr2_10cm.mprim', False, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_b_xythetamlev_test_successes = num_b_xythetamlev_test_successes + 1
    if run_sbpl_test('xythetamlev', 'adstar', 'env_examples/nav3d/env2.cfg', 'matlab/mprim/pr2_10cm.mprim', False, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_b_xythetamlev_test_successes = num_b_xythetamlev_test_successes + 1
    if run_sbpl_test('xythetamlev', 'anastar', 'env_examples/nav3d/env2.cfg', 'matlab/mprim/pr2_10cm.mprim', False, sbpl_root=sbpl_root, sbpl_exe=sbpl_exe) == 0:
        num_b_xythetamlev_test_successes = num_b_xythetamlev_test_successes + 1

    return num_b_xythetamlev_test_successes

def main():
    base_path = getcwd()

    # verify that the base path contains a test directory
    if not basename(base_path) == 'test':
        print('The specified base path "{base_path}" doesn\'t contain a test directory, '
              'this script should only be run from sbpl/test directory.'.format(base_path=base_path))
        return 1

    sbpl_root = abspath(join(base_path, ".."))

    # determine build space
    build_path = join(base_path, "build")

    # ensure build folder exists
    if not exists(build_path):
        mkdir(build_path)

    print_blue("SBPL is located at {}".format(sbpl_root))
    print_blue("Build space: {}".format(build_path))
    print_blue("#" * 33)

    configure_code = generate_makefile(build_path=build_path, source_path=sbpl_root)

    if configure_code:
        return 1

    # invoke build step
    build_code = build_project(build_path)

    if build_code:
        print_fail("Invoking {} failed".format(' '.join(cmd)))
        return 1

    sbpl_exe = join(build_path, 'bin/test_sbpl')

    sbpl_exists = exists(sbpl_exe) and \
                  exists(join(build_path, 'lib/libsbpl.so'))

    if not sbpl_exists:
        print_fail("Could not build SBPL and SBPL is not already pre-built. Aborting tests")
        exit()
    else:
        print_status("SBPL library and test executable built. Proceeding with tests.")

    num_2d_test_successes = 0;
    num_2d_test_successes = test_forward_serach_2d_environments(sbpl_root, sbpl_exe)

    num_xytheta_test_successes = 0
    num_xytheta_test_successes = test_forward_serach_xytheta_environments(sbpl_root, sbpl_exe)

    num_xythetamlev_test_successes = 0
    num_xythetamlev_test_successes = test_forward_serach_xythetamlev_environments(sbpl_root, sbpl_exe)

    num_robarm_test_successes = 0
    num_robarm_test_successes = test_forward_serach_robarm_environments(sbpl_root, sbpl_exe)

    ###### RUN ALL TESTS WITH BACKWARD SEARCH NOW ######
    print_blue("#" * 55)
    print_status("###### RUN ALL TESTS WITH BACKWARD SEARCH NOW ######")

    num_b_2d_test_successes = 0
    num_b_2d_test_successes = test_backward_serach_2d_environments(sbpl_root, sbpl_exe)

    num_b_xytheta_test_successes = 0
    num_b_xytheta_test_successes = test_bachward_serach_xytheta_environments(sbpl_root, sbpl_exe)

    num_b_xythetamlev_test_successes = 0
    num_b_xythetamlev_test_successes = test_backward_serach_xythetamlev_environments(sbpl_root, sbpl_exe)

    print()
    print_summary("Forward search results")
    print_summary("----------------------")
    print_summary("{num_2d_test_successes} out of 12 2d tests succeeded.".format(num_2d_test_successes=num_2d_test_successes))
    print_summary("{num_xytheta_test_successes} out of 9 xytheta tests succeeded.".format(num_xytheta_test_successes=num_xytheta_test_successes))
    print_summary("{num_xythetamlev_test_successes} out of 6 xythetamlev tests succeeded.".format(num_xythetamlev_test_successes=num_xythetamlev_test_successes))
    print_summary("{num_robarm_test_successes} out of 12 robarm tests succeeded.".format(num_robarm_test_successes=num_robarm_test_successes))

    num_tests = 39
    print_summary("{successed} out of {num_tests} tests succeeded.".format(successed=(num_2d_test_successes + num_xytheta_test_successes + num_xythetamlev_test_successes + num_robarm_test_successes), num_tests=num_tests))

    print()
    print_summary("Backward search results")
    print_summary("-----------------------")
    print_summary("{num_b_2d_test_successes} out of 12 2d tests succeeded.".format(num_b_2d_test_successes=num_b_2d_test_successes))
    print_summary("{num_b_xytheta_test_successes} out of 9 xytheta tests succeeded.".format(num_b_xytheta_test_successes=num_b_xytheta_test_successes))
    print_summary("{num_b_xythetamlev_test_successes} out of 6 xythetamlev tests succeeded.".format(num_b_xythetamlev_test_successes=num_b_xythetamlev_test_successes))

    num_b_tests = 27
    print_summary("{successed} out of {num_b_tests} tests succeeded.".format(successed=(num_b_2d_test_successes + num_b_xytheta_test_successes + num_b_xythetamlev_test_successes), num_b_tests=num_b_tests))
#end main

NOTES = [
    "xytheta and xythetamlev environments do not support R* planning",
    "envrobarm does not support backward search"
]

if __name__ == '__main__':
    try:
        print_red("Note:")
        for note in NOTES:
            print_blue("* " + note)
        print()

        exit(main())
    except Exception as e:
        exit(str(e))