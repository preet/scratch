1. Use blackberry-connect and pass it your public key

        blackberry-connect 192.168.x.x -password xyz -sshPublicKey id_rsa.pub

2. ssh with your (not public) ssh key and the 'devuser' account

        ssh -v -i id_rsa devuser@192.168.2.10
