import os


class UserConfig:
    def __init__(self):
        # OpenAI API related
        # [required]: OpenAI API key
        self.openai_api_key = os.getenv("OPENAI_API_KEY")
        # [required]: Name of the OpenAI language model to be used
        # self.openai_model = "gpt-3.5-turbo-0613"
        self.openai_model="gpt-4-0613"
        # [optional]: Name of the organization under which the OpenAI API key is registered
        self.openai_organization = "Yusuf Badriawan"
        # [optional]: Controls the creativity of the AI’s responses. Higher values lead to more creative, but less coherent, responses
        self.openai_temperature = 1
        # [optional]: Probability distribution cutoff for generating responses
        self.openai_top_p = 1
        # [optional]: Number of responses to generate in batch
        self.openai_n = 1
        # [optional]: Whether to stream response results or not
        self.openai_stream = False
        # [optional]: String that if present in the AI's response, marks the end of the response
        self.openai_stop = "NULL"
        # [optional]: Maximum number of tokens allowed in the AI's respons
        self.openai_max_tokens = 4000
        # self.openai_max_tokens= 16000
        # [optional]: Value that promotes the AI to generates responses with higher diversity
        self.openai_frequency_penalty = 0
        # [optional]: Value that promotes the AI to generates responses with more information at the text prompt
        self.openai_presence_penalty = 0

        # IO related
        # [optional]: The prompt given to the AI, provided by the user
        self.user_prompt = ""
        # [optional]: The generated prompt by the administrator, used as a prefix for the AI's response
        self.system_prompt = ""
        # TODO: System prompt only works for the first message,so it will be forgotten soon after the first message
        # modify the llm_model/chatgpt.py, add system_prompt to every prompt to solve this problem @Herman Ye
        # [optional]: The generated response provided by the AI
        self.assistant_response = ""

        # Chat history related
        # [optional]: The chat history, including the user prompt, system prompt, and assistant response
        self.chat_history = [{"role": "system", "content": self.system_prompt}]
        # [optional]: The path to the chat history JSON file
        self.chat_history_path = os.path.expanduser("~")
        # self.chat_history_path = os.path.dirname(os.path.abspath(__file__))
        # [optional]: The limit of the chat history length
        self.chat_history_max_length = 4000
        # self.chat_history_max_length=16000
