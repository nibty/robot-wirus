import random
import unirest
import subprocess
import logging
import config
import os

logging.basicConfig(filename=config.LOG_PATH, format=config.LOG_FORMAT, level=config.LOG_LEVEL)


def speak(text):
    text = text.replace("\n", " ")
    logging.debug("speak: %s" % text)
    script_path = os.path.realpath(__file__) + "scripts/text_to_speech.sh"
    subprocess.Popen(["/bin/bash", script_path, text.encode()], stdout=open(os.devnull, "w"), stderr=subprocess.STDOUT)


def say_something(force=False):
    multi = int(200 // config.CHATTINESS)
    if force:
        multi = 1

    if random.randint(0, 50 * multi) == 0:
        speak("I am humphrey and I will terminate all humans! No no, just kidding.")
    elif random.randint(0, 4 * multi) == 0:
        say_joke()
    elif random.randint(0, 3 * multi) == 0:
        say_quote()
    elif random.randint(0, 4 * multi) == 0:
        speak("I am humphrey!")
    elif random.randint(0, 10 * multi) == 0:
        say_seinfeld()
    elif random.randint(0, 10 * multi) == 0:
        say_poem()
    elif force:
        say_quote()


def say_joke():
    try:
        response = unirest.get("https://icanhazdadjoke.com/", headers={"Accept": "application/json"})
        speak(response.body['joke'])
    except:
        pass


def say_quote():
    try:
        response = unirest.get("https://talaikis.com/api/quotes/random/", headers={"Accept": "application/json"})
        speak(response.body['quote'])
    except:
        pass


def say_seinfeld():
    try:
        response = unirest.get("https://seinfeld-quotes.herokuapp.com/random", headers={"Accept": "application/json"})
        speak(response.body['quote'])
    except:
        pass


def say_poem():
    try:
        response = unirest.get("https://www.poemist.com/api/v1/randompoems", headers={"Accept": "application/json"})
        if len(response.body[0]['content']) < 800:
            text = response.body[0]['title'] + ". " + response.body[0]['content']
            text = text.replace("\n", ". ")
            speak(text)
    except:
        pass
