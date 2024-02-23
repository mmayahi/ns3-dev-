#!/usr/bin/env python3
"""Run ns-3 simulation campaign using SEM (Simulation Execution manager).

Defines the wrapper functions to configure and run simulation campaigns.
Currenlty focuses on performing multiple runs with same set of provided params.

"""
__author__ = "Rediet"
__copyright__ = "Orange 2023"
__version__ = "0.2"

from argparse import ArgumentParser # CLI parsing
import os # file manipulations
import sem # main lib
import subprocess # to configure ns-3 only with required modules
                  # (and reduce compilation time)
import sys # utilities


def configureCampaign(ns_path: str = "../scratch/",
                      campaign_dir: str = "/MU_logs",
                      overwrite: bool = False) -> sem.CampaignManager:
    """Configure campaign.

    Args:
        ns_path (str, optional): Relative path to the ns-3 code.
            Defaults to '../ns-3'.
        campaign_dir (str, optional): Relative path to the directory where to
            store the campaign database. Defaults to 'results'.
        overwrite (bool, optional): Whether to overwrite the campaign. Defaults
            to False.

    Returns:
        sem.CampaignManager: the CampaignManager instance (used to run
            launch the campaign downstream).
    """
    # general params
    script = 'Scratch1-twt-psm-udp-tcp'
    optimized = True

    # custom configuration options (to reduce compil time)
    custom_config = True
    modules = "wifi;buildings;flow-monitor;internet-apps;applications"
    examples = False

    # configure (if custom mode or if directory not present -> first time)
    if custom_config and not os.path.isdir(campaign_dir):
        configuration_command = ['python3', './ns3', 'configure',
                                '--enable-modules', modules,
                                '--enable-examples' if examples
                                    else '--disable-examples',
                                '--disable-gtk', '--disable-werror',
                                '--quiet']
        if optimized:
            configuration_command += ['--build-profile=optimized',
                                    '--out=build/optimized']
        configure = subprocess.run(configuration_command, cwd=ns_path,
                                   stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE)
        print(configure.stdout.decode("utf-8"))

    # build campaign
    campaign = sem.CampaignManager.new(ns_path,
                                       script,
                                       campaign_dir,
                                       # use custom config above
                                       skip_configuration=(not custom_config),
                                       overwrite=overwrite,
                                       optimized=optimized)

    print(campaign)
    return campaign


def runCampaign(campaign: sem.CampaignManager,
                randSeed: int,
                power: int,
                traffic: int,
                duration: float,
                StaCount:int,
                link: int,
                udp: bool):
    """Run simulations.

    This is more oriented multiple runs. It runs any simulations missing in the
    database.

    Args:
        campaign (sem.CampaignManager): the CampaignManager instance.
        runs (int): the number of independent runs (used to configure the seed).
        scenario (int, optional): the scenario (1 for residential and 2 for
            apartment). Defaults to 2.
        duration (float, optional): the simulation duration (in seconds).
            Defaults to 30.
        neighbors (int, optional): the number of neighboring networks. This is
            a target number. Defaults to 13.
        notepad0On6Ghz (bool, optional): whether to configure Notepad-0 on
            6GHz (5GHz if false). Defaults to True.
    """
    params = {
        'randSeed': [randSeed],
        'power': [power],
        'traffic': [traffic],
        'simulationTime': [simulationTime],
        'StaCount':[StaCount],
        'link': [link],
        'udp': [udp]
    }
    print(f"Simulated params: {params} for {randSeed} runs")
    campaign.run_missing_simulations(params, runs=runs,
                                     stop_on_errors=False) # better process all others
    print(f"There are now {len(list(campaign.db.get_complete_results()))}"
            + " results in the database")


def main():
    """CLI wrapper."""
    parser = ArgumentParser(description='Configure and run ns-3 simulation \
        campaign using SEM')
    parser.add_argument("-o", "--overwrite", dest="overwrite",
                        action="store_true", default=False,
                        help="Whether to overwrite campaign")
    parser.add_argument("-c", "--campaign_dir", dest="campaign_dir",
                        type=str,
                        default="MU_logs",
                        help="Directory where to store campaign")
    parser.add_argument("-r", "--randSeed", dest="randSeed",
                        type=int,  choices=[1],
                        help="Number of runs per combination. With different"
                             + " seed each time")
    parser.add_argument("-p", "--power", dest="power",
                        type=int, choices=[1, 2, 3], default=3,
                        help=" (power save mechanism (1 for PSM, 2 for twt and 3 for active mode)).")
    parser.add_argument("-s", "--simulationTime", dest="simulationTime",
                        type=float, default=30,
                        help="Simulation time (in seconds)")
    parser.add_argument("-t", "--traffic", dest="traffic",
                         default=1, choices=[1, 2, 3], 
                        help="traffic generator. 1: predictable periodic, 2: poisson traffic, 3: full buffer.")
    parser.add_argument("-n", "--StaCount", dest="StaCount",
                        type=int, default=1, choices=[1, 2, 3, 4, 5, 6, 7, 8, 9, 10],
                        help="Number of STAs. Integer between 1 and 10")
    parser.add_argument("-l", "--link", dest="link",
                        type=int, default=3, choices=[1, 2, 3],
                        help="communication link = 1: uplink, 2: downlink, 3: douplex")

    args = parser.parse_args()

    campaign = configureCampaign(campaign_dir=args.campaign_dir,
                                 overwrite=args.overwrite)
    runCampaign(campaign=campaign,
                runs=args.randSeed,
                scenario=args.power,
                simulationTime=args.simulationTime,
                traffic=args.traffic,
                StaCount=args.StaCount,
                link=args.link)

if __name__ == "__main__":

    # Check first that the Python version is consistent with one required
    # by SEM (i.e. >3.8 and <4).
    python_version = sys.version_info
    assert(python_version.major == 3 and python_version.minor > 8)

    main()
