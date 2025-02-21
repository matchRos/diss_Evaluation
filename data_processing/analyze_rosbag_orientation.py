import rosbag
import tf
import numpy as np
import pandas as pd
import argparse
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
import open3d as o3d
import math

# Transformation vom Messsystem ins Karten-Koordinatensystem
translation_offset = np.array([38.2691, 32.8942, 0])
rotation_angle = 0 # 3.1656  # Rotation um die Z-Achse

def quaternion_to_euler(quat):
    """ Wandelt eine Quaternion (x, y, z, w) in Euler-Winkel (Rx, Ry, Rz) um. """
    return tf.transformations.euler_from_quaternion([quat[0], quat[1], quat[2], quat[3]])

def extract_tf_data_orientations(bag_file):
    """ Extrahiert die Orientierung (Rx, Ry, Rz) aus einer ROS-Bag für virtual_object/base_link und leiter. """
    virtual_object_orientations = []
    leiter_orientations = []
    
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=['/tf']):
            for transform in msg.transforms:
                frame_id = transform.child_frame_id
                
                # Quaternion extrahieren
                quaternion = np.array([
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w
                ])
                
                # Umwandlung in Euler-Winkel
                euler_angles = np.array(quaternion_to_euler(quaternion))

                # Daten sammeln
                if frame_id == "virtual_object/base_link":
                    virtual_object_orientations.append(euler_angles)
                elif frame_id == "leiter":
                    # Korrektur der Rz-Drehung ins Karten-Koordinatensystem
                    euler_angles[0] = -euler_angles[0] 
                    euler_angles[1] = -euler_angles[1] # Invertierung der Y-Rotation
                    corrected_orientation = euler_angles + np.array([0, 0, rotation_angle])
                    leiter_orientations.append(corrected_orientation)
    
    # Umwandlung in DataFrames
    virtual_object_df = pd.DataFrame(virtual_object_orientations, columns=["Rx", "Ry", "Rz"])
    leiter_df = pd.DataFrame(leiter_orientations, columns=["Rx", "Ry", "Rz"])
    
    return virtual_object_df, leiter_df

def synchronize_orientations(virtual_object_df, leiter_df):
    """ Interpoliert BEIDE Datensätze, sodass sie auf einer gemeinsamen Zeitachse liegen. """

    # Schritt 1: Erzeuge eine gemeinsame Zeitbasis mit der höchsten Auflösung
    n_points = max(len(virtual_object_df), len(leiter_df))  # Nimmt das Maximum, nicht Minimum!
    time_common = np.linspace(0, 1, n_points)

    # Interpolation für Virtual Object (Rx, Ry, Rz)
    interp_rx_virtual = interp1d(np.linspace(0, 1, len(virtual_object_df)), virtual_object_df["Rx"], kind="linear", fill_value="extrapolate")
    interp_ry_virtual = interp1d(np.linspace(0, 1, len(virtual_object_df)), virtual_object_df["Ry"], kind="linear", fill_value="extrapolate")
    interp_rz_virtual = interp1d(np.linspace(0, 1, len(virtual_object_df)), virtual_object_df["Rz"], kind="linear", fill_value="extrapolate")

    virtual_object_interpolated = pd.DataFrame({
        "Rx": interp_rx_virtual(time_common),
        "Ry": interp_ry_virtual(time_common),
        "Rz": interp_rz_virtual(time_common)
    })

    # Interpolation für Leiter (Rx, Ry, Rz)
    interp_rx_leiter = interp1d(np.linspace(0, 1, len(leiter_df)), leiter_df["Rx"], kind="linear", fill_value="extrapolate")
    interp_ry_leiter = interp1d(np.linspace(0, 1, len(leiter_df)), leiter_df["Ry"], kind="linear", fill_value="extrapolate")
    interp_rz_leiter = interp1d(np.linspace(0, 1, len(leiter_df)), leiter_df["Rz"], kind="linear", fill_value="extrapolate")

    leiter_interpolated = pd.DataFrame({
        "Rx": interp_rx_leiter(time_common),
        "Ry": interp_ry_leiter(time_common),
        "Rz": interp_rz_leiter(time_common)
    })

    return virtual_object_interpolated, leiter_interpolated


def apply_icp_orientations(leiter_df, virtual_object_df):
    """ Führt die ICP-Registrierung für die Euler-Winkel (Rx, Ry, Rz) durch. """

    # Konvertiere die Daten in Punktwolken
    def convert_to_point_cloud(df):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(df.to_numpy())
        return pcd

    leiter_pcd = convert_to_point_cloud(leiter_df)
    virtual_object_pcd = convert_to_point_cloud(virtual_object_df)

    # ICP-Registrierung durchführen
    threshold = 0.1  # Höherer Threshold für Winkel (Radiant)
    reg_p2p = o3d.pipelines.registration.registration_icp(
        leiter_pcd, virtual_object_pcd, threshold,
        np.identity(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint()
    )

    # Transformationsmatrix
    transformation_icp = reg_p2p.transformation
    print("\n🔄 ICP-Transformation Matrix für Orientierung:")
    print(transformation_icp)

    # Transformation auf die Euler-Winkel anwenden
    leiter_pcd.transform(transformation_icp)

    # Rückgabe der transformierten Orientierungsdaten
    leiter_transformed_corrected = np.asarray(leiter_pcd.points)
    return pd.DataFrame(leiter_transformed_corrected, columns=["Rx", "Ry", "Rz"]), transformation_icp

def compute_orientation_error(virtual_object_df, leiter_df):
    """ Berechnet den Fehler zwischen der transformierten Leiter-Orientierung und dem virtuellen Objekt. """
    errors = np.abs(virtual_object_df - leiter_df)
    
    error_stats = {
        "Mean Error (Rx)": np.mean(errors["Rx"]),
        "Max Error (Rx)": np.max(errors["Rx"]),
        "RMSE (Rx)": np.sqrt(np.mean(errors["Rx"]**2)),

        "Mean Error (Ry)": np.mean(errors["Ry"]),
        "Max Error (Ry)": np.max(errors["Ry"]),
        "RMSE (Ry)": np.sqrt(np.mean(errors["Ry"]**2)),

        "Mean Error (Rz)": np.mean(errors["Rz"]),
        "Max Error (Rz)": np.max(errors["Rz"]),
        "RMSE (Rz)": np.sqrt(np.mean(errors["Rz"]**2)),
    }

    return error_stats

def plot_trajectories(virtual_object_df, leiter_df):
    """ Erstellt eine 2D-Visualisierung der Euler-Winkel Rx, Ry, Rz über der Zeit. """

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    time = np.arange(len(virtual_object_df))  # Zeitachse (Anzahl der Messpunkte)

    # Rx (Roll)
    axes[0].plot(time, virtual_object_df["Rx"], label="Virtual Object Rx", color="blue")
    axes[0].plot(time, leiter_df["Rx"], label="Leiter Rx (transformed)", color="red", linestyle="dashed")
    axes[0].set_ylabel("Rx (rad)")
    axes[0].legend()
    axes[0].grid()

    # Ry (Pitch)
    axes[1].plot(time, virtual_object_df["Ry"], label="Virtual Object Ry", color="blue")
    axes[1].plot(time, leiter_df["Ry"], label="Leiter Ry (transformed)", color="red", linestyle="dashed")
    axes[1].set_ylabel("Ry (rad)")
    axes[1].legend()
    axes[1].grid()

    # Rz (Yaw)
    axes[2].plot(time, virtual_object_df["Rz"], label="Virtual Object Rz", color="blue")
    axes[2].plot(time, leiter_df["Rz"], label="Leiter Rz (transformed)", color="red", linestyle="dashed")
    axes[2].set_ylabel("Rz (rad)")
    axes[2].set_xlabel("Zeitpunkt")
    axes[2].legend()
    axes[2].grid()

    plt.suptitle("Vergleich der Euler-Winkel Rx, Ry, Rz")
    plt.tight_layout()
    plt.show()

    # Save as PDF
    fig.savefig("euler_angle_comparison.pdf", bbox_inches='tight')

def plot_trajectories_3d(virtual_object_df, leiter_df):
    """ Erstellt eine 3D-Visualisierung der Euler-Winkel Rx, Ry, Rz als Trajektorie. """

    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')

    # Originale Euler-Winkel (Virtual Object) plotten
    ax.plot(virtual_object_df["Rx"], virtual_object_df["Ry"], virtual_object_df["Rz"], 
            label="Virtual Object Orientierung", color="blue", alpha=0.6)

    # Transformierte Euler-Winkel (Leiter) plotten
    ax.plot(leiter_df["Rx"], leiter_df["Ry"], leiter_df["Rz"], 
            label="Leiter Orientierung (transformed)", color="red", linestyle="dashed", alpha=0.6)

    ax.set_xlabel("Rx (Roll)")
    ax.set_ylabel("Ry (Pitch)")
    ax.set_zlabel("Rz (Yaw)")
    ax.set_title("Vergleich der Orientierungs-Trajektorien (Rx, Ry, Rz)")
    ax.legend()

    plt.show()

    # Save as PDF
    fig.savefig("euler_orientation_3d.pdf", bbox_inches='tight')


def main():
    parser = argparse.ArgumentParser(description="Analyse der Euler-Winkel von Leiter und Virtual Object aus einer ROS-Bag.")
    parser.add_argument("bag_file", type=str, help="Pfad zur ROS-Bag-Datei")

    args = parser.parse_args()
    virtual_object_df, leiter_df = extract_tf_data_orientations(args.bag_file)

    # Synchronisierung der Euler-Winkel
    virtual_object_df, leiter_df = synchronize_orientations(virtual_object_df, leiter_df)

    if virtual_object_df.empty or leiter_df.empty:
        print("❌ Keine relevanten /tf-Daten gefunden.")
        return
    
    # ICP-Transformation für Orientierungen anwenden
    leiter_df, transformation_matrix = apply_icp_orientations(leiter_df, virtual_object_df)

    # Visualisierung der Trajektorien
    plot_trajectories(virtual_object_df, leiter_df)
    plot_trajectories_3d(virtual_object_df, leiter_df)


    # Ausgabe der Transformationsmatrix
    print("\n🔄 ICP-Transformation Matrix für Orientierung:")
    print(transformation_matrix)

    # Fehlerberechnung
    errors = compute_orientation_error(virtual_object_df, leiter_df)
    
    if errors:
        print("\n🔍 Fehleranalyse für Euler-Winkel:")
        for key, value in errors.items():
            print(f"{key}: {value:.4f} rad")

if __name__ == "__main__":
    main()
