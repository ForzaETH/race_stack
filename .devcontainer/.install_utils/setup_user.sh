!# /bin/bash

useradd -ms /bin/bash -u ${UID} -G sudo ${USER}
groupadd -fg ${GID} ${USER}
