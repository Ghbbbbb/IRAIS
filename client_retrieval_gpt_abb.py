import os
import time
import logging
import socket
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
import json
from langchain.chains import RetrievalQA
from langchain_openai import ChatOpenAI

from prompts.prompt_template import get_qa_template_baichuan_abb
import commons.embedding_utils as eu
from commons.utils import *

logging.basicConfig(level=logging.INFO)

class GPTAssistant:
    """ Load ChatGPT config and your custom pre-prompts. """

    def __init__(self, verbose=False, prompt_doc="dcpd1_base") -> None:
        

        logging.info("Initialize LLM...")
        llm = ChatOpenAI(
            openai_api_base="https://api.gpts.vin/v1", #中转
            model="gpt-4-turbo",
            temperature=0,
            max_tokens=2048,
        )
        logging.info(f"Done.")

        logging.info("Initialize tools...")
        embedding_model = eu.init_embedding_model()
        vector_store = eu.init_vector_store(embedding_model,prompt_doc)
        logging.info(f"Done.")

        logging.info("Initialize chain...")
        chain_type_kwargs = {"prompt":get_qa_template_baichuan_abb(prompt_doc), "verbose": verbose}
        self.conversation = RetrievalQA.from_chain_type(
            llm=llm,
            chain_type='stuff',
            retriever=vector_store.as_retriever(search_kwargs={'k': 4}),
            chain_type_kwargs=chain_type_kwargs,
            return_source_documents=True
        )
        logging.info(f"Done.")

        os.system("clear")
        streaming_print_banner()

    def ask(self, question):
        result_dict = self.conversation(question)
        result = result_dict['result']
        return result

class VoiceCommandHandler(FileSystemEventHandler):
    def __init__(self, gpt, socket):
        self.gpt = gpt
        self.socket = socket
    
    def on_created(self, event):
        if event.is_directory:
            return
        if event.src_path.endswith('.txt'):
            time.sleep(0.1)  # 延迟以确保文件写入完成
            with open(event.src_path, 'r+', encoding='utf-8') as file:
                command = file.read().strip()
                if command:
                    result = self.gpt.ask(command)
                    self.socket.sendall(result.encode())
                    print('\033[31m'"question:", command)
                    print('\033[32m'"result:", result, '\n')
                    file_path = os.path.join('Iphone-response', "result.txt")
                    with open(file_path, "a", encoding="utf-8") as file:
                        file.write("question: " + command + "\n")
                        file.write("result: " + result + "\n\n")
                    print('\033[37m'f"Questions and results saved to {file_path}")

                    # 等待确认消息
                    ack = self.socket.recv(1024).decode()
                    if ack != "ACK":
                        logging.warning("Did not receive ACK from server")
                else:
                    print("Empty command detected, ignoring.")

def main(IS_DEBUG=False, IS_DOC=False, IS_VOICE=False, PROMPT_DOC = "dcpd1_base"):
    os.environ["TOKENIZERS_PARALLELISM"] = "false"
    gpt = GPTAssistant(verbose=True,prompt_doc=PROMPT_DOC)
    
    if not IS_DEBUG:
        HOST = 'localhost'
        PORT = 5001
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((HOST, PORT))
        print("Connected to server.")
    else:
        s = None

    if IS_VOICE:
        path = "Iphone-voice"  # 监控文件夹
        event_handler = VoiceCommandHandler(gpt, s)
        observer = Observer()
        observer.schedule(event_handler, path, recursive=False)
        observer.start()
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            observer.stop()
        observer.join()
    elif IS_DOC:
        with open("E:/Git/Git/gitclone/MRoP/dataset/dcpd1-1.json", "r") as file:
            data = json.load(file)
            num_entries = len(data)
        start = time.time()
        for entry in data:
            question = entry.get("instruction")  # 获取问题
            result = gpt.ask(question)  # 调用函数获取答案
            print('\033[31m'"question:", question)
            print('\033[32m'"result:", result, '\n')

            if not IS_DEBUG:
                s.sendall(result.encode())
                # 等待确认消息
                ack = s.recv(1024).decode()
                if ack != "ACK":
                    logging.warning("Did not receive ACK from server")
        end = time.time()
        print("mean_time:", (end - start) / num_entries)
    else:
        while True:
            question = input("User> ")
            if question == "!quit" or question == "!exit":
                break
            if question == "!clear":
                os.system("cls")
                continue

            result = gpt.ask(question)  # Ask a question
            print("Assistant> " + f"{result}")
            if not IS_DEBUG:
                s.sendall(result.encode())
                # 等待确认消息
                ack = s.recv(1024).decode()
                if ack != "ACK":
                    logging.warning("Did not receive ACK from server")

    if not IS_DEBUG and s:
        s.shutdown(socket.SHUT_RDWR)
        s.close()

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description="Client for interacting with robot via various input methods.")
    parser.add_argument("--debug", action="store_true", help="Run in debug mode")
    parser.add_argument("--doc", action="store_true", help="Run in document input mode")
    parser.add_argument("--voice", action="store_true", help="Run in voice command mode")
    parser.add_argument("--prompt", type=str, default="IRAIS", help="Specify the prompt directory")
    args = parser.parse_args()
    main(IS_DEBUG=args.debug, IS_DOC=args.doc, IS_VOICE=args.voice, PROMPT_DOC=args.prompt)
