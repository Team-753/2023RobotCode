py -3 -m robotpy_installer download-python
py -3 -m robotpy_installer download robotpy[all] robotpy-ctre robotpy-navx robotpy-rev robotpy-apriltag robotpy-photonvision robotpy-pathplannerlib robotpy-commands-v2 robotpy-playingwithfusion pynetworktables
echo please disconnect from wifi and connect to your robot
PAUSE
py -3 -m robotpy_installer install-python
py -3 -m robotpy_installer install robotpy[all] robotpy-ctre robotpy-navx robotpy-rev robotpy-apriltag robotpy-photonvision robotpy-pathplannerlib robotpy-commands-v2 robotpy-playingwithfusion pynetworktables
echo done
PAUSE