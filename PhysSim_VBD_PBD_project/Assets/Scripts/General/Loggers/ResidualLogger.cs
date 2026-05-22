using System;
using System.Globalization;
using System.IO;
using UnityEngine;

public class ResidualLogger : MonoBehaviour
{
    public bool writeCsv = true;
    public bool showOverlay = true;
    public string label = "Sim";
    public float overlayY = 90f;

    public Func<ResidualSample> Sampler { get; set; }

    private StreamWriter csv;
    private string csvPath;
    private float simTime;
    private int frameNumber;
    private ResidualSample latest;
    private GUIStyle style;

    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    void Start()
    {
        if (writeCsv) OpenCsv();
    }

    void OnDestroy() { CloseCsv(); }
    void OnApplicationQuit() { CloseCsv(); }

    public void Log(float dt)
    {
        latest = Sampler();
        frameNumber++;
        simTime += dt;
        if (csv != null)
            csv.WriteLine(string.Format(Inv,
                "{0};{1:F6};{2:G6};{3:G6};{4:G6}",
                frameNumber, simTime,
                latest.maxForce, latest.avgForce, latest.maxAccel));
    }

    void OnGUI()
    {
        if (!showOverlay) return;
        if (style == null)
        {
            style = new GUIStyle(GUI.skin.label) { fontSize = 14 };
            style.normal.textColor = Color.white;
        }
        GUI.Label(new Rect(10, overlayY, 700, 20),
            string.Format(Inv,
                "{0} residual  maxF={1:G4}  avgF={2:G4}  maxAccel={3:G4} m/s^2",
                label, latest.maxForce, latest.avgForce, latest.maxAccel),
            style);
    }

    private void OpenCsv()
    {
        var dir = Path.GetFullPath(Path.Combine(Application.dataPath, "../../Data-analysis/Logs/ResidualLogs"));
        Directory.CreateDirectory(dir);
        string safe = string.IsNullOrEmpty(label) ? "Sim" : label;
        foreach (char c in Path.GetInvalidFileNameChars()) safe = safe.Replace(c, '_');
        csvPath = Path.Combine(dir, $"{safe}_{DateTime.Now:yyyyMMdd_HHmmss}.csv");
        csv = new StreamWriter(csvPath, false);
        csv.WriteLine("frame;time;maxForce;avgForce;maxAccel"); // force in N, accel in m/s^2, time in s
        Debug.Log($"[ResidualLogger] {label}: writing residual log to {csvPath}");
    }

    private void CloseCsv()
    {
        if (csv == null) return;
        csv.Flush();
        csv.Close();
        csv = null;
    }
}
