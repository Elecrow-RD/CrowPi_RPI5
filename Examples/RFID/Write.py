from mfrc522 import SimpleMFRC522

reader = SimpleMFRC522()


text = "Hello World"
reader.write(text)
print("Written")
