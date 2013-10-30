cmake_minimum_required(VERSION 2.8.3)
project(mcr_speech_msgs)

find_package(catkin REQUIRED message_generation)

add_message_files(
  FILES
   RecognizedSpeech.msg  
   Say.msg
)

add_service_files(
   FILES
   ChangeGrammar.srv  
   SetRecognitionConfidence.srv
)

generate_messages()

catkin_package()
