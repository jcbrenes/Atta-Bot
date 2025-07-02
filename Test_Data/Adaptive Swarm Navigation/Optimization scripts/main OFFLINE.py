import ADPDic_Data
import os
import re
import csv

def get_filtered_files_sorted_by_modified_time(directory, pattern):
    # Compile the regex pattern
    regex = re.compile(pattern)
    
    # List all files in the given directory and filter them using the regex pattern
    files = [os.path.join(directory, f) for f in os.listdir(directory) if os.path.isfile(os.path.join(directory, f)) and regex.match(f)]
    
    # Get the last modified time for each file and sort them
    files.sort(key=lambda x: os.path.getmtime(x))
    
    return files

file_pattern = r'^divided_color_matrix_\d{2}-\d{2}-\d{2}\.csv$'  # Regex pattern for the desired files
#raiz_path = '../Processing/mapeo/'

# Aqui es donde se pone la carpeta específica donde están TODAS las pruebas
#raiz_path = "D:/Pruebas de optimización/"
raiz_path = "D:/Pruebas_Optimizacion_Offline_V2/"
DataDang = 'C:/Users/jcbre/OneDrive - TEC/GIRoM/Paper Opt Nav/Pruebas_Optimizacion/MatrizPeligro.csv'
#por facilidad para pruebas sucesivas, diferencio que las carpetas y archivos se creen en otro path
#destino_path = "C:/Users/juanbrenes/OneDrive - TEC/GIRoM/Paper Opt Nav/Pruebas_Optimizacion/"
destino_path = "D:/Pruebas_Optimizacion_Offline_V2/"


# Iterate through existing reduced color matrices
for cantidadRobots in range(1, 4):
    for tiempoPrueba in [5, 10, 15]:
        for prueba in range(1, 14):
            try:
                # Construct paths for reduced and divided matrices
                divided_matrix_path = f"{raiz_path}{cantidadRobots}_robot/{tiempoPrueba}-{prueba}/divided_color_matrix/"
                Cob_path = f"{destino_path}{cantidadRobots}_robot/{tiempoPrueba}-{prueba}/path_objective_Cob.csv"
                final_path = f"{destino_path}{cantidadRobots}_robot/{tiempoPrueba}-{prueba}/"
                
                # Creo carpetas para cada tipo de archivo
                #os.mkdir(final_path)
                os.mkdir(final_path +"A/")
                os.mkdir(final_path +"D/")
                os.mkdir(final_path +"P/")
                os.mkdir(final_path +"Dict/")
                os.mkdir(final_path +"Graph/")
                
                    
                # Intentamos recuperar todos los archivos de pruebas
                try:
                    sorted_filtered_files = get_filtered_files_sorted_by_modified_time(divided_matrix_path, file_pattern)
                    
                    # Abro un solo archivo para ir guardando en cada fila el path, objective y Cob
                    with open(Cob_path, 'w', newline='', encoding='utf-8') as csvfile:
                        writer_path = csv.writer(csvfile)
                        
                        # Cuarto loop para revisar cada matriz de la prueba
                        for DataFile in sorted_filtered_files:
                            
                            # Extraigo la fecha de la prueba para agregársela a los archivos de optimización
                            filename = os.path.basename(DataFile)  # Get the filename from the full path
                            match = re.search(r'^divided_color_matrix_(\d{2}-\d{2}-\d{2})\.csv$', filename)                
                            if match:    
                                fecha_prueba = match.group(1)
                                archivo_A = final_path +"A/A_" + fecha_prueba + ".csv"                        
                                archivo_D = final_path +"D/D_" + fecha_prueba + ".csv"
                                archivo_P = final_path +"P/P_" + fecha_prueba + ".csv"
                                archivo_Dict = final_path +"Dict/Dict_" + fecha_prueba + ".json"
                                archivo_Graph = final_path +"Graph/Graph_" + fecha_prueba + ".png"
                                
                                # Ahora sí, se hace la optimización
                                print("\nOptimizando archivo " + DataFile + "...\n")
                                [A,D,P,Net,F,C,Cob] = ADPDic_Data.GenData_ADPDic(DataFile,DataDang,archivo_A,archivo_D,archivo_P,archivo_Dict);
                                
                                try:
                                    path_Opt = ADPDic_Data.Graph_Opt(A,Net,F,C,DataDang,True,archivo_Graph,False,0);
                                    writer_path.writerow([fecha_prueba] + list(path_Opt[0]['path']) + list(path_Opt[0]['objective']) + [Cob])
                                except ValueError as error:
                                    print(error,"\n\n")
                                    pass
                    
                # Si la prueba no existe (no se ha realizado) se la salta
                except FileNotFoundError:
                    print(f"\nLa prueba {cantidadRobots}_robot/{tiempoPrueba}-{prueba} no existe")
                    pass             
                
            except FileExistsError:
                print(f"\nArchivo {cantidadRobots}_robot/{tiempoPrueba}-{prueba} saltado porque ya se optimizó.")
                pass