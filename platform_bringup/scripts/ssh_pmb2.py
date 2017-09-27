


import paramiko
ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(
    paramiko.AutoAddPolicy())

ssh.connect('pmb2-5c', username='pal', 
    password='pal')
'''
stdin, stdout, stderr = ssh.exec_command(
    "sudo ntpdate -u -b pmb2-5c", get_pty=True)
'''
stdin, stdout, stderr = ssh.exec_command(
    "source ~/catkin_gr_ws/devel/setup.bash; rosrun openni_tracker2 openni_tracker2", get_pty=True)


'''
#data = stdout.read.splitlines()

'''

data = stdout.readlines()
for line in data:
    if line.split(':')[0] == 'AirPort':
        print line
