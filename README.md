## **Dependencies**

1. Install ros packages for webots

`$ sudo apt-get install ros-noetic-webots-ros`

## **bashrc Setting**

1. open ~/.bashrc with any text editor
2. Add WEBOTS_HOME path

```bash
export WEBOTS_HOME=/usr/local/webots
```

3. Add Webots's controller library path to LD_LIBRARY_PATH

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$WEBOTS_HOME/lib/controller
```

4. Add Webots's python controller library path to PYTHONPATH
   
```bash
export PYTHONPATH=$PYTHONPATH:$WEBOTS_HOME/lib/controller/python
```

[//]: <makefile, properties, bash> 
