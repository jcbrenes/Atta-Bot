import ADPDic_Data
import os
import re
import csv
import time
from datetime import datetime

def get_filtered_files_sorted_by_modified_time(directory, pattern):
    # Compile the regex pattern
    regex = re.compile(pattern)
    
    # List all files in the given directory and filter them using the regex pattern
    files = [os.path.join(directory, f) for f in os.listdir(directory) if os.path.isfile(os.path.join(directory, f)) and regex.match(f)]
    
    # Get the last modified time for each file and sort them
    files.sort(key=lambda x: os.path.getmtime(x))
    
    return files

file_pattern = r'^divided_color_matrix_\d{2}-\d{2}-\d{2}\.csv$'  # Regex pattern for the desired files

# Aqui es donde se pone la carpeta específica donde están TODAS las pruebas
root_path = "C:/Users/Isaac/Desktop/Github Repos/PROE/Optimizacion de rutas/Processing"
final_path = root_path + "/mapeo/prueba/"
divided_matrix_path = final_path + "divided_color_matrix"
Cob_path = final_path + "/path_objective_Cob.csv"
DataDang = 'MatrizPeligro.csv'

# Creo carpetas para cada tipo de archivo
try:
    os.mkdir(final_path + "A/")
    os.mkdir(final_path + "D/")
    os.mkdir(final_path + "P/")
    os.mkdir(final_path + "Dict/")
    os.mkdir(final_path + "Graph/")
except FileExistsError:
    pass

flag_primera_iteracion = True
processed_files = set()  # Set to keep track of processed files

try:
    with open(Cob_path, 'w', newline='', encoding='utf-8') as csvfile:
        writer_path = csv.writer(csvfile)
        start_time = time.time()  # Tomo el tiempo a partir del cuál empezó la optimización
        
        while True:
            sorted_filtered_files = get_filtered_files_sorted_by_modified_time(divided_matrix_path, file_pattern)
            DataFile = sorted_filtered_files[-1]  # Get the most recent file
                
            if DataFile not in processed_files:  # Check if the file has already been processed
                processed_files.add(DataFile)  # Add the file to the set of processed files
                
                # Extraigo la fecha de la prueba para agregársela a los archivos de optimización
                filename = os.path.basename(DataFile)  # Get the filename from the full path
                match = re.search(r'^divided_color_matrix_(\d{2}-\d{2}-\d{2})\.csv$', filename)
                
                if match:
                    fecha_prueba = match.group(1)
                    archivo_A = final_path + "A/A_" + fecha_prueba + ".csv"
                    archivo_D = final_path + "D/D_" + fecha_prueba + ".csv"
                    archivo_P = final_path + "P/P_" + fecha_prueba + ".csv"
                    archivo_Dict = final_path + "Dict/Dict_" + fecha_prueba + ".json"
                    archivo_Graph = final_path + "Graph/Graph_" + fecha_prueba + ".png"
                    
                    # Ahora sí, se hace la optimización
                    print("\nOptimizando...\n")
                    [A, D, P, Net, F, C, Cob] = ADPDic_Data.GenData_ADPDic(DataFile, DataDang, archivo_A, archivo_D, archivo_P, archivo_Dict)
                    
                    
                    if flag_primera_iteracion:
                        time.sleep(10)
                        flag_primera_iteracion = False
                    
                    try:
                        path_Opt = ADPDic_Data.Graph_Opt(A, Net, F, C, DataDang, True, archivo_Graph, True, 0)
                        writer_path.writerow(list(path_Opt[0]['path']) + list(path_Opt[0]['objective']) + [Cob])
                        print(f"¡Lista optimización de {DataFile}!\n")
                        
                    except ValueError as error:
                        print(error, "\n")
                        pass
                
            #else:
                #print("¡La actual prueba ya está optimizada!")
            
except KeyboardInterrupt:
    print("Programa finalizado por el usuario.")
