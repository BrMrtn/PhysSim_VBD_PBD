using System;
using System.Globalization;
using System.IO;
using UnityEngine;

public class AmplitudeLogger : MonoBehaviour
{
    public bool writeCsv = true;
    public bool showOverlay = true;
    public string label = "Sim";
    public float overlayY = 90f;

    public Func<AmplitudeSample> Sampler { get; set; }

    private StreamWriter csv;
    private string csvPath;
    private float simTime;
    private int frameNumber;
    private AmplitudeSample latest;
    private Vector3 startEnd;   // end position captured on the first sample (reference)
    private bool hasReference;
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
        if (!hasReference)
        {
            startEnd = latest.endPosition;
            hasReference = true;
        }
        frameNumber++;
        simTime += dt;

        // Signed displacement of the weighted end from its starting position.
        // For a downwards-stretched chain the oscillation axis is y; the |disp| envelope is the amplitude.
        Vector3 disp = latest.endPosition - startEnd;

        if (csv != null)
            csv.WriteLine(string.Format(Inv,
                "{0};{1:F6};{2:F6};{3:F6};{4:F6};{5:F6}",
                frameNumber, simTime,
                disp.x, disp.y, disp.z, disp.magnitude));
    }

    void OnGUI()
    {
        if (!showOverlay) return;
        if (style == null)
        {
            style = new GUIStyle(GUI.skin.label) { fontSize = 14 };
            style.normal.textColor = Color.white;
        }
        Vector3 disp = latest.endPosition - startEnd;
        GUI.Label(new Rect(10, overlayY, 700, 20),
            string.Format(Inv,
                "{0} amplitude  dx={1:F3}  dy={2:F3}  dz={3:F3}  |d|={4:F3}",
                label, disp.x, disp.y, disp.z, disp.magnitude),
            style);
    }

    private void OpenCsv()
    {
        var dir = Path.GetFullPath(Path.Combine(Application.dataPath, "../../Data/AmplitudeLogs"));
        Directory.CreateDirectory(dir);
        string safe = string.IsNullOrEmpty(label) ? "Sim" : label;
        foreach (char c in Path.GetInvalidFileNameChars()) safe = safe.Replace(c, '_');
        csvPath = Path.Combine(dir, $"{safe}_{DateTime.Now:yyyyMMdd_HHmmss}.csv");
        csv = new StreamWriter(csvPath, false);
        csv.WriteLine("frame;time;dx;dy;dz;displacement"); // displacement in meters, time in seconds
        Debug.Log($"[AmplitudeLogger] {label}: writing amplitude log to {csvPath}");
    }

    private void CloseCsv()
    {
        if (csv == null) return;
        csv.Flush();
        csv.Close();
        csv = null;
    }
}
