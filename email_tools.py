from os import system
from datetime import datetime

import imaplib
from smtplib import SMTP
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.image import MIMEImage
from time import sleep

from gc_constants import EMAIL_SENDER_ADDR, EMAIL_SENDER_PSWD

def wait_for_email(searchAddr):
    mail = imaplib.IMAP4_SSL("imap.gmail.com")
    mail.login(EMAIL_SENDER_ADDR, EMAIL_SENDER_PSWD)
    mail.list()

    count = 0
    while count < 60:
        try:
            mail.select("inbox")
            result, data = mail.search(None, "(UNSEEN FROM " + searchAddr + ")")
            print("{0}, {1}".format(len(data), result))
            result, data = mail.fetch(data[0].split()[-1], "(RFC822)")
            if data is None:
                print("Waiting...")
            else:
                print("Process initiated!")
                break
        except IndexError:
            sleep(2)
            count += 1
    else:
        print("Exceeded timeout.")

def send_images(to, cc, snapshot_locations=[]):
    recipientAddrs = to + cc
    if len(recipientAddrs) <= 0:
        print("Did not receive any recipients!")
        return

    snapshot_timestamp = datetime.now().strftime("%Y%m%d%H%M%S")
    if len(snapshot_locations) == 0:
        snapshot_locations.append(snapshot_timestamp + ".jpg")
        snapshot_command = "raspistill -w 1280 -h 720 -vf -hf -o " + snapshot_location
        system(snapshot_command)

    subject = "Image(s) sent at " + snapshot_timestamp

    msg = MIMEMultipart()
    msg["Subject"] = subject
    msg["From"] = EMAIL_SENDER_ADDR
    msg["To"] = ",".join(to)
    msg["CC"] = ",".join(cc)

    msg.preamble = subject

    msg.attach(MIMEText(subject))

    for snapshot_location in snapshot_locations:
        with open(snapshot_location, "rb") as fd:
            msg.attach(MIMEImage(fd.read()))

    smtp = SMTP("smtp.gmail.com", 587)
    smtp.ehlo()
    smtp.starttls()
    smtp.ehlo()
    smtp.login(EMAIL_SENDER_ADDR, EMAIL_SENDER_PSWD)
    smtp.sendmail(EMAIL_SENDER_ADDR, recipientAddrs, msg.as_string())
    smtp.quit()
