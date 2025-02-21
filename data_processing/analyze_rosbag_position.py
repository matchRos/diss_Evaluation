import rosbag
import tf
import numpy as np
import pandas as pd
import argparse
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import interp1d
import open3d as o3d

# Transformation vom Messsystem ins Karten-Koordinatensystem
translation_offset = np.array([39.2691-0.1710, 31.8942-0.0634, 0])
rotation_angle = 3.1656  # Rotation um die Z-Achse

def rotate_z(points, angle):
    """ Rotiert Punkte um die Z-Achse. """
    R = np.array([
        [np.cos(angle), -np.sin(angle), 0],
        [np.sin(angle),  np.cos(angle), 0],
        [0,              0,             1]
    ])
    return points @ R.T

def extract_tf_data_positions(bag_file):
    """ Extrahiert die Positionen von 'virtual_object/base_link' und 'leiter' aus einer ROS-Bag. """
    virtual_object_positions = []
    leiter_positions = []
    
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=['/tf']):
            for transform in msg.transforms:
                frame_id = transform.child_frame_id
                
                # Position extrahieren
                position = np.array([
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z
                ])
                
                # Daten sammeln
                if frame_id == "virtual_object/base_link":
                    virtual_object_positions.append(position)
                elif frame_id == "leiter":
                    # Transformation ins Karten-Koordinatensystem
                    transformed_position = rotate_z(position - translation_offset, rotation_angle)
                    leiter_positions.append(transformed_position)
    
    # Umwandlung in DataFrames
    virtual_object_df = pd.DataFrame(virtual_object_positions, columns=["x", "y", "z"])
    leiter_df = pd.DataFrame(leiter_positions, columns=["x", "y", "z"])
    
    return virtual_object_df, leiter_df

def synchronize_positions(virtual_object_df, leiter_df):
    """ Interpoliert BEIDE Trajektorien, sodass sie die gleiche Anzahl an Punkten haben. """
    
    # Gemeinsame Zeitbasis mit maximaler Auflösung
    n_points = max(len(virtual_object_df), len(leiter_df))
    time_common = np.linspace(0, 1, n_points)

    # Interpolation für Virtual Object (x, y, z)
    interp_x_virtual = interp1d(np.linspace(0, 1, len(virtual_object_df)), virtual_object_df["x"], kind="linear", fill_value="extrapolate")
    interp_y_virtual = interp1d(np.linspace(0, 1, len(virtual_object_df)), virtual_object_df["y"], kind="linear", fill_value="extrapolate")
    interp_z_virtual = interp1d(np.linspace(0, 1, len(virtual_object_df)), virtual_object_df["z"], kind="linear", fill_value="extrapolate")

    virtual_object_interpolated = pd.DataFrame({
        "x": interp_x_virtual(time_common),
        "y": interp_y_virtual(time_common),
        "z": interp_z_virtual(time_common)
    })

    # Interpolation für Leiter (x, y, z)
    interp_x_leiter = interp1d(np.linspace(0, 1, len(leiter_df)), leiter_df["x"], kind="linear", fill_value="extrapolate")
    interp_y_leiter = interp1d(np.linspace(0, 1, len(leiter_df)), leiter_df["y"], kind="linear", fill_value="extrapolate")
    interp_z_leiter = interp1d(np.linspace(0, 1, len(leiter_df)), leiter_df["z"], kind="linear", fill_value="extrapolate")

    leiter_interpolated = pd.DataFrame({
        "x": interp_x_leiter(time_common),
        "y": interp_y_leiter(time_common),
        "z": interp_z_leiter(time_common)
    })

    return virtual_object_interpolated, leiter_interpolated

def compute_error(virtual_object_df, leiter_df):
    """ Berechnet den Fehler zwischen der transformierten Leiter-Trajektorie und dem virtuellen Objekt. """
    if len(virtual_object_df) != len(leiter_df):
        print("⚠️ Achtung: Unterschiedliche Anzahl an Zeitpunkten! Die Daten sollten synchronisiert werden.")
        return None

    errors = np.linalg.norm(virtual_object_df.to_numpy() - leiter_df.to_numpy(), axis=1)
    
    error_stats = {
        "Mean Error": np.mean(errors),
        "Max Error": np.max(errors),
        "RMSE": np.sqrt(np.mean(errors**2)),
    }

    # Fehler pro Achse
    error_stats.update({
        "Mean Error (x)": np.mean(np.abs(virtual_object_df["x"] - leiter_df["x"])),
        "Max Error (x)": np.max(np.abs(virtual_object_df["x"] - leiter_df["x"])),
        "RMSE (x)": np.sqrt(np.mean((virtual_object_df["x"] - leiter_df["x"])**2)),

        "Mean Error (y)": np.mean(np.abs(virtual_object_df["y"] - leiter_df["y"])),
        "Max Error (y)": np.max(np.abs(virtual_object_df["y"] - leiter_df["y"])),
        "RMSE (y)": np.sqrt(np.mean((virtual_object_df["y"] - leiter_df["y"])**2)),

        "Mean Error (z)": np.mean(np.abs(virtual_object_df["z"] - leiter_df["z"])),
        "Max Error (z)": np.max(np.abs(virtual_object_df["z"] - leiter_df["z"])),
        "RMSE (z)": np.sqrt(np.mean((virtual_object_df["z"] - leiter_df["z"])**2)),
    })

    return error_stats

def plot_trajectories_2d(virtual_object_df, leiter_df):
    """ Erstellt eine 2D-Visualisierung der Positionsfehler über die Zeit. """

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    time = np.arange(len(virtual_object_df))

    # x-Fehler
    axes[0].plot(time, virtual_object_df["x"], label="Virtual Object x", color="blue")
    axes[0].plot(time, leiter_df["x"], label="Leiter x (transformed)", color="red", linestyle="dashed")
    axes[0].set_ylabel("x (m)")
    axes[0].legend()
    axes[0].grid()

    # y-Fehler
    axes[1].plot(time, virtual_object_df["y"], label="Virtual Object y", color="blue")
    axes[1].plot(time, leiter_df["y"], label="Leiter y (transformed)", color="red", linestyle="dashed")
    axes[1].set_ylabel("y (m)")
    axes[1].legend()
    axes[1].grid()

    # z-Fehler
    axes[2].plot(time, virtual_object_df["z"], label="Virtual Object z", color="blue")
    axes[2].plot(time, leiter_df["z"], label="Leiter z (transformed)", color="red", linestyle="dashed")
    axes[2].set_ylabel("z (m)")
    axes[2].set_xlabel("Zeitpunkt")
    axes[2].legend()
    axes[2].grid()

    plt.suptitle("Vergleich der Positionen x, y, z")
    plt.tight_layout()
    plt.show()

    # Save as PDF
    fig.savefig("position_comparison_2d.pdf", bbox_inches='tight')

def plot_trajectories_3d(virtual_object_df, leiter_df):
    """ Erstellt eine 3D-Visualisierung der Trajektorien. """
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')

    # Originale Trajektorie (Virtual Object)
    ax.plot(virtual_object_df["x"], virtual_object_df["y"], virtual_object_df["z"], 
            label="Virtual Object", color="blue", alpha=0.6)

    # Transformierte Leiter-Trajektorie
    ax.plot(leiter_df["x"], leiter_df["y"], leiter_df["z"], 
            label="Leiter (transformed)", color="red", linestyle="dashed", alpha=0.6)

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("Vergleich der Positions-Trajektorien (x, y, z)")
    ax.legend()

    plt.show()
    fig.savefig("position_comparison_3d.pdf", bbox_inches='tight')

def apply_icp(leiter_df, virtual_object_df):
    """ Führt die ICP-Registrierung durch, um die bestmögliche Transformation zu bestimmen. """

    # Konvertiere die Daten in Punktwolken
    def convert_to_point_cloud(df):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(df.to_numpy())
        return pcd

    leiter_pcd = convert_to_point_cloud(leiter_df)
    virtual_object_pcd = convert_to_point_cloud(virtual_object_df)

    # ICP-Registrierung durchführen
    threshold = 0.2  # Maximale Distanz für das Matching
    reg_p2p = o3d.pipelines.registration.registration_icp(
        leiter_pcd, virtual_object_pcd, threshold,
        np.identity(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint()
    )

    # Transformationsmatrix
    transformation_icp = reg_p2p.transformation
    print("\n🔄 ICP-Transformation Matrix:")
    print(transformation_icp)

    # Transformation auf die Leiter-Daten anwenden
    leiter_pcd.transform(transformation_icp)

    # Rückgabe der transformierten Daten
    leiter_transformed_corrected = np.asarray(leiter_pcd.points)
    return pd.DataFrame(leiter_transformed_corrected, columns=["x", "y", "z"]), transformation_icp


def main():
    parser = argparse.ArgumentParser(description="Analyse der Positionsdaten aus einer ROS-Bag.")
    parser.add_argument("bag_file", type=str, help="Pfad zur ROS-Bag-Datei")

    args = parser.parse_args()
    virtual_object_df, leiter_df = extract_tf_data_positions(args.bag_file)

    # Synchronisierung der Daten
    virtual_object_df, leiter_df = synchronize_positions(virtual_object_df, leiter_df)

    # ICP-Transformation anwenden
    leiter_df, transformation_matrix = apply_icp(leiter_df, virtual_object_df)

    # Fehleranalyse
    plot_trajectories_2d(virtual_object_df, leiter_df)
    plot_trajectories_3d(virtual_object_df, leiter_df)

    errors = compute_error(virtual_object_df, leiter_df)
    
    if errors:
        print("\n🔍 Fehleranalyse:")
        for key, value in errors.items():
            print(f"{key}: {value:.4f} m")

if __name__ == "__main__":
    main()
