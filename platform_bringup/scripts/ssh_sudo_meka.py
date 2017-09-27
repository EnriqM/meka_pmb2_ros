


import paramiko


comand = "sudo ntpdate -u -b pmb2-5c"
#comand = "m3rt_server_run -m"
ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(
    paramiko.AutoAddPolicy())

ssh.connect('meka-man', username='meka', 
    password='mekameka')

stdin, stdout, stderr = ssh.exec_command( comand, get_pty=True)

stdin.write('mekameka\n')
stdin.flush()


#data = stdout.read.splitlines()


data = stdout.readlines()
for line in data:
    if line.split(':')[0] == 'AirPort':
        print line



'''


import paramiko, getpass, re, time


host = 'meka-man'

ssh_client = paramiko.SSHClient()
ssh_client.set_missing_host_key_policy(
    paramiko.AutoAddPolicy()) 
ssh_client.connect( 'meka-man', username='meka', 
    password='mekameka' )
sudo_pw = 'mekameka' #getpass.getpass("sudo pw for %s: " % host)
command = "sudo ntpdate -u -b pmb2-5c"

channel = ssh_client.invoke_shell() 
channel.send( command )       
# wait for prompt             
while not re.search(".*\[sudo\].*",channel.recv(1024)): time.sleep(1)
channel.send( "%s\n" % sudo_pw )

'''
