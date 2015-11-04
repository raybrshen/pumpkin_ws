#List of requested files from the file_lister that is inside an determined folder
#This message is intended to be used inside Files srv.
#filenames: List of filenames
string[] filenames
#folder: The folder that the files are inside
string folder
#parent_folder: An number identifing the index of the parent folder
#(as this message is inside a vector)
uint8 parent_folder
