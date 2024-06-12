from mfrc522 import SimpleMFRC522

reader = SimpleMFRC522()


text = "64"
reader.write(text)
print("Written")
