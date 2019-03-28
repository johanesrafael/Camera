#!/usr/bin/env python3

"""Hello World

Make Vector say 'Hello World' in this simple Vector SDK example program.
"""

import anki_vector
import os
import sys
import time
try:
    from PIL import Image
except ImportError:
    sys.exit("Cannot import from PIL: Do `pip3 install --user Pillow` to install")

def show_given_image(image_name, duration=1.0):
    """
    The images have to be in the custom_images folder
    They have to be JPG
    :param image_name:
    :param duration: How long the image stays there
    :return:
    """
    with anki_vector.Robot() as robot:
        current_directory = os.path.dirname(os.path.realpath(__file__))
        image_path = os.path.join(current_directory, "custom_images", image_name)

        # Load an image
        image_file = Image.open(image_path)

        # Convert the image to the format used by the Screen
        print("Display image on Vector's face...")
        screen_data = anki_vector.screen.convert_image_to_screen_data(image_file)
        robot.screen.set_screen_with_image_data(screen_data, duration)


def play_given_image_animation(image_animation_name):
    """
    The images have to be in the custom_images folder
    images have to have format 184x96
    :param image_name:
    :return:
    """
    with anki_vector.Robot() as robot:
        current_directory = os.path.dirname(os.path.realpath(__file__))
        image_path = os.path.join(current_directory, "custom_images", image_animation_name)

        # Get list of frames:
        from os import listdir
        from os.path import isfile, join
        framefiles_list = [f for f in listdir(image_path) if isfile(join(image_path, f))]

        for frame_image in framefiles_list:
            # Load an image
            frame_image_path = os.path.join(image_path,frame_image)
            image_file = Image.open(frame_image_path)

            # Convert the image to the format used by the Screen
            print("Display image on Vector's face...")
            screen_data = anki_vector.screen.convert_image_to_screen_data(image_file)
            robot.screen.set_screen_with_image_data(screen_data, 0.04)



def print_animation_list():

    anim_list = []

    with anki_vector.Robot() as robot:
        print("List all animation names:")
        anim_names = robot.anim.anim_list
        for anim_name in anim_names:
            print(anim_name)
            anim_list.append(anim_name)

    return anim_list

def execute_list_animation(animation_list):
    with anki_vector.Robot() as robot:

        for animation in animation_list:
            print("Playing animation by name: " + animation)
            robot.anim.play_animation(animation)


def say_frases(text_to_say):
    args = anki_vector.util.parse_command_args()
    with anki_vector.Robot(args.serial) as robot:

        for frase in text_to_say:
            print(frase)
            robot.say_text(frase)


def main_episode_2():

    animation_list = [  "anim_reacttoblock_ask_01_0",
                        "anim_keepalive_eyes_01_forward",
                        "anim_gazing_lookatvector_reaction_01_head_angle_20",
                        "anim_holiday_hyn_confetti_01",
                        "anim_nocloud_icon_01",
                        "anim_keepalive_eyes_01_right",
                        "anim_eyecontact_lookloop_01",
                        "anim_eyepose_happy",
                        "anim_onboarding_reacttoface_happy_01_head_angle_40"
                      ]

    text_to_say_1 = ["Thanks Mike! Hi Internet Humans!",
                   "So to setup your local computer to be able to execute programs in myself and do things like this"]

    say_frases(text_to_say_1)
    play_given_image_animation("eye_animation")

    text_to_say_2 = ["You will need to follow these three simple steps.",
                        "Step 1, Create a virtual environment",
                        "We will first create a folder where to store all the Vector code you do from now on.",
                        "To create a virtual environment you execute these commands."
                        "They will create an environment where the python version used is 3.6, because that's the one I like."
                     ]

    say_frases(text_to_say_2)
    execute_list_animation([animation_list[0]])

    text_to_say_3 = ["Step 2, You have to install my SDK",
                    "This depends on the operating system that you use. These are the commands for ubuntu 16, but in the video description I will leave a link to all the other installation options",
                    "The most important step here is retrieving your own vector name, ID and IP.",
                    "This is done by double click my backpack. Here you will get my name.",
                    "Then, move my arms up and down to get my IP and my ID or ESN.",
                    "Once you have them you will have to introduce them when asked in the anki_vector configure script."
                    "Also don't forget to install the 3D viewer if you want to execute some of the tutorials."
                    ]

    say_frases(text_to_say_3)
    execute_list_animation([animation_list[2]])

    text_to_say_4 = ["Step 3,  download the anki tutorials.",
                    "Just execute these commands.",
                    "Before executing any python code, you have to set the pythonpath to get the open cv python version for python 3.6. This is critical especially if you have ROS installed, because it generated a lot of problems if not. Basically it wont find the cv2 python method.",
                    "Lets execute hello world"
                    ]

    say_frases(text_to_say_4)
    execute_list_animation([animation_list[5]])

    text_to_say_5 = ["And that is it for today.",
                     "On the next video we will install and execute a ROS package designed to integrate my systems with ROS."
                     ]

    say_frases(text_to_say_5)
    show_given_image("roslogo.jpg", duration=3.0)

    text_to_say_6 = [" This was created by beta bot.",
                     "Very very nice guy!"]
    say_frases(text_to_say_6)
    show_given_image("beta_bot_person.jpg", duration=3.0)

    text_to_say_7 = ["Don't Forget to subscribe!",
                     "It will be fun."]

    say_frases(text_to_say_7)
    play_given_image_animation("rick_morty_wow")

    text_to_say_8 = ["Until then.",
                     "Keep Building!"]

    say_frases(text_to_say_8)
    show_given_image("EndScreen.jpg", duration=10.0)

    execute_list_animation([animation_list[7],animation_list[8]])

if __name__ == "__main__":
    main_episode_2()



