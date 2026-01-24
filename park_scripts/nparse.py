#!/usr/bin/env python3

###############################################################################
# Copyright 2024 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################
"""
nparse.py: parse and upload performance results dumped by cyber_performance

Usage：
    1. login 
        > buildtool login USERNAME PASSWORD

    2. only show the results
       > python3 nparse.py -f performance_dumps.06-20-2024.json -v 9.0.0-rc-r15 -i dkit_advance_arm
        or upload the results
       > python3 nparse.py -r -f performance_dumps.06-20-2024.json -v 9.0.0-rc-r15 -i dkit_advance_arm
"""

import argparse
import subprocess
import os
import re
import sys
import json
import numpy as np
import requests
from datetime import datetime

VERSION = None
PERFORMANCE_FILE = None
VEHICLE_ID = None
REPORT_RESULT = False
API_PREFIX = "https://apollo.baidu.com"
JOB_API = "/packages/api/job"
DETAIL_API = "/packages/api/job_detail"

TOKEN_PATH = "/opt/apollo/neo/share/buildtool/id_token"
TOKEN = ""
if os.path.exists(TOKEN_PATH):
    with open(TOKEN_PATH, "r") as f:
        TOKEN = f.read().strip()

HEADERS = {
    "Host": "apollo.baidu.com",
    "Version": "9.0.0-rc-r1-upload-script",
    "Authorization": "Bearer {}".format(TOKEN)
}
FORMAT_STR = "%m/%d/%Y, %H:%M:%S"

MERTIC_TO_UPLOAD_KEY = {
    "cpu_usage": "cpu",
    "gpu_usage": "gpu",
    "memory": "mem",
    "gpu_memory": "gpu_mem",
    "io_wait_usage": "io_wait",
    "block_device_io_read": "io_read",
    "block_device_io_write": "io_write",
    "ethernet_device_io_read": "eth_io_read",
    "ethernet_device_io_write": "eth_io_write", 
}

PERFORMANCE_ALL = {}
PERFORMANCE_AUTODRIVE = {}
DETAILS = {}

def get_upload_field(mertic_name):
    """get the field name of upload json"""
    if mertic_name in MERTIC_TO_UPLOAD_KEY:
        return MERTIC_TO_UPLOAD_KEY[mertic_name]
    return None

def parse_result():
    """
    parse result read from dumped json file of cyber_performance
    """
    global PERFORMANCE_FILE
    global PERFORMANCE_ALL
    global PERFORMANCE_AUTODRIVE
    raw_result = []

    if not os.path.exists(PERFORMANCE_FILE):
        print(f"[ERROR] {PERFORMANCE_FILE} not exists!")
        exit(1)
    with open(PERFORMANCE_FILE, "r") as f:
        raw_result = f.readlines()

    for raw in raw_result:
        try:
            sample = json.loads(raw)
        except:
            continue
        for process in sample["data"]:
            if process not in PERFORMANCE_ALL:
                PERFORMANCE_ALL[process] = {
                    "BASIC": {
                        "cpu_usage": [],
                        "gpu_usage": [],
                        "memory": [],
                        "gpu_memory": [],
                    },
                    "BLOCK_DEVICE_IO": {
                        "io_wait_usage": [],
                        "block_device_io_read": [],
                        "block_device_io_write": [],
                        "devices_io_raw": {},
                    },
                    "ETHERNET_DEVICE_IO": {
                        "ethernet_device_io_read": [],
                        "ethernet_device_io_write": [],
                        "devices_io_raw": {},
                    },
                    "E2E_LATENCY": {},
                    "time": []
                }
            if process not in PERFORMANCE_AUTODRIVE:
                PERFORMANCE_AUTODRIVE[process] = {
                    "BASIC": {
                        "cpu_usage": [],
                        "gpu_usage": [],
                        "memory": [],
                        "gpu_memory": [],
                    },
                    "BLOCK_DEVICE_IO": {
                        "io_wait_usage": [],
                        "block_device_io_read": [],
                        "block_device_io_write": [],
                        "devices_io_raw": {},
                    },
                    "ETHERNET_DEVICE_IO": {
                        "ethernet_device_io_read": [],
                        "ethernet_device_io_write": [],
                        "devices_io_raw": {},
                    },
                    "E2E_LATENCY": {},
                    "time": []
                }
            autodrive_status = sample["data"][process]["autodrive"]
            for i in range(len(autodrive_status)):
                if autodrive_status[i]:
                    for monitor_ele in sample["data"][process]:
                        if monitor_ele == "autodrive":
                            continue
                        if monitor_ele.startswith("time"):
                            PERFORMANCE_AUTODRIVE[process]["time"].append(
                                sample["data"][process][monitor_ele][i])
                            continue
                        
                        monitor_ele_list = monitor_ele.split(" - ")
                        metric_type = monitor_ele_list[0]
                        metric_device = None
                        metric_value = None
                        if len(monitor_ele_list) > 2:
                            metric_device = monitor_ele_list[1]
                            metric_name = re.sub(r'\(.*?\)', '', monitor_ele_list[2])
                        else:
                            metric_name = re.sub(r'\(.*?\)', '', monitor_ele_list[1])
                        if metric_type not in PERFORMANCE_AUTODRIVE[process]:
                            continue
                        if metric_type == "BASIC":
                            PERFORMANCE_AUTODRIVE[process][metric_type][metric_name].append(
                                float(sample["data"][process][monitor_ele][i]))
                        elif metric_type == "E2E_LATENCY":
                            try:
                                if metric_name not in PERFORMANCE_AUTODRIVE[process][metric_type]:
                                    PERFORMANCE_AUTODRIVE[process][metric_type][metric_name] = []
                                PERFORMANCE_AUTODRIVE[process][metric_type][metric_name].append(
                                    float(sample["data"][process][monitor_ele][i]))
                            except:
                                continue
                        else:
                            if metric_device is None or metric_device == "system":
                                PERFORMANCE_AUTODRIVE[process][metric_type][metric_name].append(
                                    float(sample["data"][process][monitor_ele][i]))
                            else:
                                if metric_device not in \
                                        PERFORMANCE_AUTODRIVE[process][metric_type]["devices_io_raw"]:
                                    PERFORMANCE_AUTODRIVE[process][metric_type]["devices_io_raw"][metric_device] = {}
                                if metric_name not in PERFORMANCE_AUTODRIVE[
                                        process][metric_type]["devices_io_raw"][metric_device]:
                                    PERFORMANCE_AUTODRIVE[process][
                                        metric_type]["devices_io_raw"][metric_device][metric_name] = []
                                PERFORMANCE_AUTODRIVE[process][metric_type]["devices_io_raw"][
                                    metric_device][metric_name].append(float(sample["data"][process][monitor_ele][i]))
                        
                for monitor_ele in sample["data"][process]:
                    if monitor_ele == "autodrive":
                        continue
                    if monitor_ele.startswith("time"):
                        PERFORMANCE_ALL[process]["time"].append(
                            sample["data"][process][monitor_ele][i])
                        continue
                    
                    monitor_ele_list = monitor_ele.split(" - ")
                    metric_type = monitor_ele_list[0]
                    metric_device = None
                    metric_value = None
                    if len(monitor_ele_list) > 2:
                        metric_device = monitor_ele_list[1]
                        metric_name = re.sub(r'\(.*?\)', '', monitor_ele_list[2])
                    else:
                        metric_name = re.sub(r'\(.*?\)', '', monitor_ele_list[1])
                    if metric_type not in PERFORMANCE_ALL[process]:
                        continue
                    if metric_type == "BASIC":
                        PERFORMANCE_ALL[process][metric_type][metric_name].append(
                            float(sample["data"][process][monitor_ele][i]))
                    elif metric_type == "E2E_LATENCY":
                        try:
                            if metric_name not in PERFORMANCE_ALL[process][metric_type]:
                                PERFORMANCE_ALL[process][metric_type][metric_name] = []
                            PERFORMANCE_ALL[process][metric_type][metric_name].append(
                                float(sample["data"][process][monitor_ele][i]))
                        except:
                            continue
                    else:
                        if metric_device is None or metric_device == "system":
                            PERFORMANCE_ALL[process][metric_type][metric_name].append(
                                float(sample["data"][process][monitor_ele][i]))
                        else:
                            if metric_device not in \
                                    PERFORMANCE_ALL[process][metric_type]["devices_io_raw"]:
                                PERFORMANCE_ALL[process][metric_type]["devices_io_raw"][metric_device] = {}
                            if metric_name not in PERFORMANCE_ALL[
                                    process][metric_type]["devices_io_raw"][metric_device]:
                                PERFORMANCE_ALL[process][metric_type]["devices_io_raw"][metric_device][metric_name] = []
                            PERFORMANCE_ALL[process][metric_type]["devices_io_raw"][metric_device][metric_name].append(
                                float(sample["data"][process][monitor_ele][i]))
    if REPORT_RESULT:
        return
    print("Performance: ")
    for p in PERFORMANCE_ALL:
        print(f"\t{p}:")
        for ele in PERFORMANCE_ALL[p]:
            if ele == "time":
                continue
            print(f"\t\t{ele}:")
            for mertic in PERFORMANCE_ALL[p][ele]:
                if mertic == "devices_io_raw":
                    for dev in PERFORMANCE_ALL[p][ele][mertic]:
                        print(f"\t\t\t{dev}:")
                        for dev_metric in PERFORMANCE_ALL[p][ele][mertic][dev]:
                            avg = round(
                                np.mean(PERFORMANCE_ALL[p][ele][mertic][dev][dev_metric]), 2)
                            percentile_90 = round(
                                np.percentile(PERFORMANCE_ALL[p][ele][mertic][dev][dev_metric], 90), 2)
                            print(f"\t\t\t\t{dev_metric}:")
                            print(f"\t\t\t\t\tavg: {avg}")
                            print(f"\t\t\t\t\t90th: {percentile_90}")
                else:
                    if len(PERFORMANCE_ALL[p][ele][mertic]) == 0:
                        continue
                    avg = round(np.mean(PERFORMANCE_ALL[p][ele][mertic]), 2)
                    percentile_90 = round(np.percentile(PERFORMANCE_ALL[p][ele][mertic], 90), 2)
                    print(f"\t\t\t{mertic}:")
                    print(f"\t\t\t\tavg: {avg}")
                    print(f"\t\t\t\t90th: {percentile_90}")
            
    print("\n")
    print("Performance during autodrive: ")
    for p in PERFORMANCE_AUTODRIVE:
        print(f"\t{p}:")
        for ele in PERFORMANCE_AUTODRIVE[p]:
            if ele == "time":
                continue
            print(f"\t\t{ele}:")
            for mertic in PERFORMANCE_AUTODRIVE[p][ele]:
                if mertic == "devices_io_raw":
                    for dev in PERFORMANCE_AUTODRIVE[p][ele][mertic]:
                        print(f"\t\t\t{dev}:")
                        for dev_metric in PERFORMANCE_AUTODRIVE[p][ele][mertic][dev]:
                            avg = round(
                                np.mean(PERFORMANCE_AUTODRIVE[p][ele][mertic][dev][dev_metric]), 2)
                            percentile_90 = round(
                                np.percentile(PERFORMANCE_AUTODRIVE[p][ele][mertic][dev][dev_metric], 90), 2)
                            print(f"\t\t\t\t{dev_metric}:")
                            print(f"\t\t\t\t\tavg: {avg}")
                            print(f"\t\t\t\t\t90th: {percentile_90}")
                else:
                    if len(PERFORMANCE_AUTODRIVE[p][ele][mertic]) == 0:
                        continue
                    avg = round(np.mean(PERFORMANCE_AUTODRIVE[p][ele][mertic]), 2)
                    percentile_90 = round(np.percentile(PERFORMANCE_AUTODRIVE[p][ele][mertic], 90), 2)
                    print(f"\t\t\t{mertic}:")
                    print(f"\t\t\t\tavg: {avg}")
                    print(f"\t\t\t\t90th: {percentile_90}")

def upload():
    """
    upload the parse result to database
    """
    job_request_data = {
        "jobs": []
    }
    all_avg_performance = {}
    all_90per_performance = {}
    autodrive_avg_performance = {}
    autodrive_90per_performance = {}

    for p in PERFORMANCE_ALL:
        all_avg_performance[p] = {}
        all_90per_performance[p] = {}
        for metric_type in PERFORMANCE_ALL[p]:
            latency = False
            if metric_type == "time":
                continue
            if metric_type == "BLOCK_DEVICE_IO":
                dev_type = "block"
            elif metric_type == "ETHERNET_DEVICE_IO":
                dev_type = "eth"
            elif metric_type == "E2E_LATENCY":
                latency = True

            for mertic in PERFORMANCE_ALL[p][metric_type]:
                if get_upload_field(mertic) is None and latency is False:
                    all_avg_performance[p][f"{dev_type}_{mertic}"] = {}
                    all_90per_performance[p][f"{dev_type}_{mertic}"] = {} 
                    for dev in PERFORMANCE_ALL[p][metric_type][mertic]:
                        all_avg_performance[p][f"{dev_type}_{mertic}"][dev] = {}
                        all_90per_performance[p][f"{dev_type}_{mertic}"][dev] = {}
                        for dev_metric in PERFORMANCE_ALL[p][metric_type][mertic][dev]:
                            all_avg_performance[p][f"{dev_type}_{mertic}"][dev][dev_metric] = \
                                round(np.mean(PERFORMANCE_ALL[p][metric_type][mertic][dev][dev_metric]), 2)
                            all_90per_performance[p][f"{dev_type}_{mertic}"][dev][dev_metric] = \
                                round(np.percentile(PERFORMANCE_ALL[p][metric_type][mertic][dev][dev_metric], 90), 2)
                elif get_upload_field(mertic) is None and latency is True:
                    all_avg_performance[p][f"{mertic}"] = round(
                        np.mean(PERFORMANCE_ALL[p][metric_type][mertic]), 2)
                    all_90per_performance[p][f"{mertic}"] = round(
                        np.percentile(PERFORMANCE_ALL[p][metric_type][mertic], 90), 2)
                else:
                    if len(PERFORMANCE_ALL[p][metric_type][mertic]) == 0:
                        continue
                    all_avg_performance[p][get_upload_field(mertic)] = round(
                        np.mean(PERFORMANCE_ALL[p][metric_type][mertic]), 2)
                    all_90per_performance[p][get_upload_field(mertic)] = round(
                        np.percentile(PERFORMANCE_ALL[p][metric_type][mertic], 90), 2)
                         
    job_request_data["jobs"].append({
        "pkg_version": VERSION,
        "vehicle_in": VEHICLE_ID,
        "run_date": PERFORMANCE_FILE.split(".")[-2],
        "road_type": 1,
        "is_auto": 0,
        "performances": all_avg_performance
    })
    job_request_data["jobs"].append({
        "pkg_version": VERSION,
        "vehicle_in": VEHICLE_ID,
        "run_date": PERFORMANCE_FILE.split(".")[-2],
        "road_type": 2,
        "is_auto": 0,
        "performances": all_90per_performance
    })
    
    for p in PERFORMANCE_AUTODRIVE:
        autodrive_avg_performance[p] = {}
        autodrive_90per_performance[p] = {}
        for metric_type in PERFORMANCE_AUTODRIVE[p]:
            latency = False
            if metric_type == "time":
                continue
            if metric_type == "BLOCK_DEVICE_IO":
                dev_type = "block"
            elif metric_type == "ETHERNET_DEVICE_IO":
                dev_type = "eth"
            elif metric_type == "E2E_LATENCY":
                latency = True

            for mertic in PERFORMANCE_AUTODRIVE[p][metric_type]:
                if get_upload_field(mertic) is None and latency is False:
                    autodrive_avg_performance[p][f"{dev_type}_{mertic}"] = {}
                    autodrive_90per_performance[p][f"{dev_type}_{mertic}"] = {} 
                    for dev in PERFORMANCE_AUTODRIVE[p][metric_type][mertic]:
                        autodrive_avg_performance[p][f"{dev_type}_{mertic}"][dev] = {}
                        autodrive_90per_performance[p][f"{dev_type}_{mertic}"][dev] = {}
                        for dev_metric in PERFORMANCE_AUTODRIVE[p][metric_type][mertic][dev]:
                            autodrive_avg_performance[p][f"{dev_type}_{mertic}"][dev][dev_metric] = \
                                round(np.mean(PERFORMANCE_AUTODRIVE[p][metric_type][mertic][dev][dev_metric]), 2)
                            autodrive_90per_performance[p][f"{dev_type}_{mertic}"][dev][dev_metric] = \
                                round(np.percentile(
                                    PERFORMANCE_AUTODRIVE[p][metric_type][mertic][dev][dev_metric], 90), 2)         
                elif get_upload_field(mertic) is None and latency is True:
                    try:
                        autodrive_avg_performance[p][f"{mertic}"] = round(
                            np.mean(PERFORMANCE_AUTODRIVE[p][metric_type][mertic]), 2)
                        autodrive_90per_performance[p][f"{mertic}"] = round(
                            np.percentile(PERFORMANCE_AUTODRIVE[p][metric_type][mertic], 90), 2)
                    except:
                        autodrive_avg_performance[p][f"{mertic}"] = 0
                        autodrive_90per_performance[p][f"{mertic}"] = 0 
                else:
                    if len(PERFORMANCE_AUTODRIVE[p][metric_type][mertic]) == 0:
                        continue
                    autodrive_avg_performance[p][get_upload_field(mertic)] = round(
                        np.mean(PERFORMANCE_AUTODRIVE[p][metric_type][mertic]), 2)
                    autodrive_90per_performance[p][get_upload_field(mertic)] = round(
                        np.percentile(PERFORMANCE_AUTODRIVE[p][metric_type][mertic], 90), 2)

    job_request_data["jobs"].append({
        "pkg_version": VERSION,
        "vehicle_in": VEHICLE_ID,
        "run_date": PERFORMANCE_FILE.split(".")[-2],
        "road_type": 1,
        "is_auto": 1,
        "performances": autodrive_avg_performance
    })
    job_request_data["jobs"].append({
        "pkg_version": VERSION,
        "vehicle_in": VEHICLE_ID,
        "run_date": PERFORMANCE_FILE.split(".")[-2],
        "road_type": 2,
        "is_auto": 1,
        "performances": autodrive_90per_performance
    })
    
    job_detail_request_data = {
        "jobs_detail": []
    }
    for p in PERFORMANCE_ALL:
        time_map = {}
        for index in range(len(PERFORMANCE_ALL[p]["time"])):
            if PERFORMANCE_ALL[p]["time"][index] not in time_map:
                time_map[PERFORMANCE_ALL[p]["time"][index]] = True
            else:
                continue
            ele = {
                "pkg_version": VERSION,
                "vehicle_in": VEHICLE_ID,
                "module": p,
                "start_time": datetime.strptime(
                    PERFORMANCE_ALL[p]["time"][index], FORMAT_STR).strftime("%Y-%m-%d %H:%M:%S"),
            }
            for metric_type in PERFORMANCE_ALL[p]:
                latency = False
                if metric_type == "time":
                    continue
                if metric_type == "BLOCK_DEVICE_IO":
                    dev_prefix = "block"
                elif metric_type == "ETHERNET_DEVICE_IO":
                    dev_prefix = "eth"
                elif metric_type == "E2E_LATENCY":
                    dev_prefix = "latency"
                    latency = True
                for mertic in PERFORMANCE_ALL[p][metric_type]:
                    if latency:
                        continue
                    elif get_upload_field(mertic) is not None:
                        if len(PERFORMANCE_ALL[p][metric_type][mertic]) == 0:
                            continue
                        ele[get_upload_field(mertic)] = \
                            PERFORMANCE_ALL[p][metric_type][mertic][index]
                    else:
                        if f"{dev_prefix}_{mertic}" not in ele:
                            ele[f"{dev_prefix}_{mertic}"] = {}
                        for dev in PERFORMANCE_ALL[p][metric_type][mertic]:
                            if dev not in ele[f"{dev_prefix}_{mertic}"]:
                                ele[f"{dev_prefix}_{mertic}"][dev] = {}
                            for dev_metric in PERFORMANCE_ALL[p][metric_type][mertic][dev]:
                                ele[f"{dev_prefix}_{mertic}"][dev][dev_metric] = \
                                    PERFORMANCE_ALL[p][metric_type][mertic][dev][dev_metric][index]

            job_detail_request_data["jobs_detail"].append(ele)

    res = requests.post(f"{API_PREFIX}{JOB_API}", json=job_request_data, headers=HEADERS)
    if res.status_code != 200 or res.json()["code"] != 200:
        print(f"upload job failed: {res.text}")
        exit(-1)
    res = requests.post(f"{API_PREFIX}{DETAIL_API}", json=job_detail_request_data, headers=HEADERS)
    if res.status_code != 200 or res.json()["code"] != 200:
        print(f"upload details failed: {res.text}")
        exit(-1)

def main():
    """ Main Method
    """
    global VERSION
    global PERFORMANCE_FILE
    global VEHICLE_ID
    global REPORT_RESULT

    parser = argparse.ArgumentParser(
        description='performance results parse and upload tool')
    parser.add_argument("-f", "--file", required=True,
        nargs=1, type=str.lstrip, help="the performance file dumped by cyber_performance")
    parser.add_argument("-v", "--version", required=True,
        nargs=1, type=str.lstrip, help="package version")
    parser.add_argument("-i", "--identify", required=True,
        nargs=1, type=str.lstrip, help="vehicle identified code")
    parser.add_argument('-r', '--report', default=False,
                        help='report the performance result to database', action="store_true")

    args = parser.parse_args()
    
    VERSION = args.version[0]
    PERFORMANCE_FILE = args.file[0]
    VEHICLE_ID = args.identify[0]
    REPORT_RESULT = args.report

    parse_result()

    if REPORT_RESULT:
        upload()


if __name__ == '__main__':
    main()
