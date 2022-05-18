---
layout: page
title: CLI
permalink: /cli
nav_order: 3
---

# Command Line Interface (CLI) 
{: .no_toc }
FogROS 2 uses Python entry points to extend the commands and verbs for ROS 2's CLI. Specifically, the `fogros2` package adds the `fog` command with three verbs, `list`, `connect` and `delete` for easy viewing of, interaction with, and termination of remote instances, respectively. More information about commands or verbs can be found by running `ros2 fog --help` or `ros2 fog <verb> --help`.

<details open markdown="block">
  <summary>
    CLI Commands
  </summary>
  {: .text-delta }
  - TOC
{:toc}
</details>

## list
Prints information about existing FogROS 2 instances.

### Usage
**ros2 fog list** *[-h] [\-\-region REGION [REGION ...]]*

### Description
Prints a list of details -- instance name, cloud service provider (currently always AWS), region, instance type, region, id, IP address, SSH key, disk size, AMI, and state -- for existing FogROS 2 instances. With no *REGION* specified, this command defaults to listing instances in the region set in the user's AWS configuration or in the corresponding environment variable. Otherwise, it lists all FogROS 2 instances in *REGION* or in multiple regions if specified.

### Options
**\-h, \-\-help**  
Print usage, argument, and options information and exit.

**\-\-region** *REGION [REGION ...]*  
Specify the AWS region(s) for listing instances (overrides config or environment variables that are set). Multiple regions can be listed and separated by a space, and the output will include instances from all regions listed.

### Examples
If there are two current instances running with names "great-bulkhead" and "associative-singularity":  
```bash
$ ros2 fog list 
====== great-bulkhead ======
cloud_service_provider: AWS  
ec2_region: None  
ec2_instance_type: t2.micro  
ec2_instance_id: i-002391eea4a22a6ba  
public_ip: 13.57.237.42  
ssh_key: FogROS2KEY-great-bulkhead  
disk_size: 30  
aws_ami_image: ami-00f25057ddc9b310b  
state: running  
====== associative-singularity ======  
cloud_service_provider: AWS  
ec2_region: None  
ec2_instance_type: t2.micro  
ec2_instance_id: i-071770e8a9f6d18e9  
public_ip: 18.144.35.60  
ssh_key: FogROS2KEY-associative-singularity  
disk_size: 30  
aws_ami_image: ami-00f25057ddc9b310b  
state: running  
```

One could also receive the same output as above by specifying the region directly:  
```bash
$ ros2 fog list --region us-west-1
```

## connect
Opens a shell connection to an existing instance.

### Usage
**ros2 fog connect** *[\-h] [\-\-region [REGION [REGION ...]]] [\-\-user [USER]] NAME*

### Description
Connects via SSH to an existing FogROS 2 instance with name *NAME*, allowing the user to run commands from a shell directly on the instance. FogROS 2 instance names can be found using the `list` command. If no instances with name *NAME* are found in the specified region, this command will print "No matching instance found" and exit.

### Options
**\-h, \-\-help**  
Print usage, argument, and options information and exit.

**\-\-region** *REGION [REGION ...]*  
Match only instances with name *NAME* in the AWS region(s) *REGION* (overrides config or environment variables that are set). Multiple regions can be listed and separated by a space, and all regions will be searched for an instance with name *NAME*.

**\-u** *USER*, **\-\-user** *USER*  
Connect as user *USER* to the remote SSH instance. If this option is not specified, the default user is "ubuntu".

### Examples
To connect to a currently running instance called "great-bulkhead":  
```bash
$ ros2 fog connect great-bulkhead
Welcome to Ubuntu 20.04.3 LTS (GNU/Linux 5.11.0-1022-aws x86_64)

 * Documentation:  https://help.ubuntu.com  
 * Management:     https://landscape.canonical.com  
 * Support:        https://ubuntu.com/advantage  

 System information disabled due to load higher than 1.0


231 updates can be applied immediately.  
163 of these updates are standard security updates.  
To see these additional updates run: apt list --upgradable  


Last login: Thu Feb 24 19:38:54 2022 from 128.32.37.69  
ubuntu@ip-172-31-7-52:~$
```

*Note: Output is given as an example, but likely will be different from above.*  
*Note: A notice saying that the authenticity of the host can't be established may be shown; you will need to confirm that you wish to continue to connect in this case.*

## delete
Terminates a running instance.

### Usage
**ros2 fog delete** *[\-h] [\-\-region [REGION [REGION ...]]] [\-\-dry-run] NAME*

### Description
Terminates an existing FogROS 2 instance with name *NAME* and removes the key pair and data folder associated with the instance. If *NAME* is `all`, then all FogROS 2 instances in the specified regions will be terminated. FogROS 2 instance names can be found using the `list` command. If no instances with name *NAME* are found in the specified region, this command will print "No EC2 instances found with the specified name; check list to be sure name is correct!
No instances deleted" and exit.

### Options
**\-\-dry-run**  
Show which instances and keys would be terminated and deleted without actually executing the commands to do so.

**\-h, \-\-help**  
Print usage, argument, and options information and exit.

**\-\-region** *REGION [REGION ...]*  
Match only instances with name *NAME* in the AWS region(s) *REGION* (overrides config or environment variables that are set). Multiple regions can be listed and separated by a space, and all regions will be searched for an instance with name *NAME*.

### Examples
The following examples assume there are two instances running with names "great-bulkhead" and "associative-singularity".

To terminate both running instances:
```bash
$ ros2 fog delete all
Deleting great-bulkhead i-002391eea4a22a6ba  
    terminating instance i-002391eea4a22a6ba  
    deleting key pair FogROS2KEY-great-bulkhead  
    done.  
Deleting associative-singularity i-071770e8a9f6d18e9  
    terminating instance i-071770e8a9f6d18e9  
    deleting key pair FogROS2KEY-associative-singularity  
    done.  
```

To terminate just "great-bulkhead":
```bash
$ ros2 fog delete great-bulkhead
Deleting great-bulkhead i-002391eea4a22a6ba  
    terminating instance i-002391eea4a22a6ba  
    deleting key pair FogROS2KEY-great-bulkhead  
    done. 
```