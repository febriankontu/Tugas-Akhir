import 'dart:async';
import 'dart:math';
import 'package:flutter/material.dart';
import 'package:cloud_firestore/cloud_firestore.dart';
import 'package:maps_toolkit/maps_toolkit.dart' as mp;

class HitungLahanPage extends StatefulWidget {
  const HitungLahanPage({super.key});

  @override
  State<HitungLahanPage> createState() => _HitungLahanPageState();
}

class _HitungLahanPageState extends State<HitungLahanPage> {
  final FirebaseFirestore _firestore = FirebaseFirestore.instance;
  StreamSubscription<DocumentSnapshot>? _gpsSubscription;

  // Live Data
  double? currentLat;
  double? currentLng;
  String statusGPS = "Menunggu Sinyal...";
  Color statusColor = Colors.orange;

  // Data Lahan
  final List<Map<String, double>> _perimeterPoints = []; // Titik Keliling (Batas)
  List<mp.LatLng> _generatedGridPoints = []; // Titik Sampling Hasil Generate

  double luasArea = 0.0;

  // Konfigurasi Jarak Antar Titik (Grid Distance)
  // 0.0001 derajat kira-kira 11 meter di khatulistiwa
  double stepSize = 0.00015; // Sekitar 15 Meter antar titik

  @override
  void initState() {
    super.initState();
    _startListeningGPS();
  }

  @override
  void dispose() {
    _gpsSubscription?.cancel();
    super.dispose();
  }

  void _startListeningGPS() {
    _gpsSubscription?.cancel();
    _gpsSubscription = _firestore.collection('Data_testing').doc('live_data').snapshots().listen((snapshot) {
      if (!mounted) return;
      if (snapshot.exists && snapshot.data() != null) {
        var data = snapshot.data() as Map<String, dynamic>;
        setState(() {
          currentLat = (data['latitude'] is num) ? (data['latitude'] as num).toDouble() : null;
          currentLng = (data['longitude'] is num) ? (data['longitude'] as num).toDouble() : null;

          if (currentLat != null && currentLat != 0.0) {
            statusGPS = "GPS Siap";
            statusColor = Colors.green;
          }
        });
      }
    });
  }

  void _rekamBatas() {
    if (currentLat == null) return;
    setState(() {
      _perimeterPoints.add({"lat": currentLat!, "lng": currentLng!});
    });
  }

  // --- LOGIKA 1 & 2: HITUNG LUAS & BUAT GRID ---
  void _kalkulasiDanGenerateGrid() {
    if (_perimeterPoints.length < 3) {
      ScaffoldMessenger.of(context).showSnackBar(const SnackBar(content: Text("Minimal 3 titik batas!")));
      return;
    }

    // 1. Hitung Luas
    List<mp.LatLng> polygon = _perimeterPoints.map((p) => mp.LatLng(p['lat']!, p['lng']!)).toList();
    double area = mp.SphericalUtil.computeArea(polygon).toDouble().abs();

    // 2. Generate Grid (Bounding Box)
    double minLat = polygon.first.latitude;
    double maxLat = polygon.first.latitude;
    double minLng = polygon.first.longitude;
    double maxLng = polygon.first.longitude;

    for (var p in polygon) {
      if (p.latitude < minLat) minLat = p.latitude;
      if (p.latitude > maxLat) maxLat = p.latitude;
      if (p.longitude < minLng) minLng = p.longitude;
      if (p.longitude > maxLng) maxLng = p.longitude;
    }

    List<mp.LatLng> tempGrid = [];

    // Loop membuat titik grid di dalam kotak batas
    for (double lat = minLat; lat <= maxLat; lat += stepSize) {
      for (double lng = minLng; lng <= maxLng; lng += stepSize) {
        mp.LatLng point = mp.LatLng(lat, lng);

        // Cek apakah titik ada DI DALAM Polygon lahan
        bool inside = mp.PolygonUtil.containsLocation(point, polygon, false);
        if (inside) {
          tempGrid.add(point);
        }
      }
    }

    // Jika grid kosong (lahan terlalu kecil), ambil titik tengah (centroid) sederhana
    if (tempGrid.isEmpty) {
      tempGrid.add(mp.LatLng((minLat+maxLat)/2, (minLng+maxLng)/2));
    }

    setState(() {
      luasArea = area;
      _generatedGridPoints = tempGrid;
    });

    _tampilkanDialogGrid();
  }

  void _tampilkanDialogGrid() {
    if (!mounted) return;
    showModalBottomSheet(
      context: context,
      builder: (context) => Container(
        padding: const EdgeInsets.all(20),
        child: Column(
          children: [
            const Icon(Icons.grid_on, size: 40, color: Colors.blue),
            const Text("Rencana Sampling", style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold)),
            const Divider(),
            Text("Luas Lahan: ${luasArea.toStringAsFixed(0)} m²"),
            Text("Jarak Antar Titik: ~${(stepSize * 111000).toStringAsFixed(0)} Meter"),
            const SizedBox(height: 10),
            Text(
                "${_generatedGridPoints.length} TITIK SAMPLE DIPERLUKAN",
                style: const TextStyle(fontSize: 24, fontWeight: FontWeight.bold, color: Colors.green)
            ),
            const SizedBox(height: 10),
            const Text("Silakan lihat daftar titik di bawah dan datangi satu per satu untuk ambil sampel.", textAlign: TextAlign.center, style: TextStyle(color: Colors.grey)),
          ],
        ),
      ),
    );
  }

  void _reset() {
    setState(() {
      _perimeterPoints.clear();
      _generatedGridPoints.clear();
      luasArea = 0;
    });
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text("1. Buat Peta Grid"), backgroundColor: Colors.blue[800], foregroundColor: Colors.white),
      body: Column(
        children: [
          // Header GPS
          Container(
            padding: const EdgeInsets.all(12),
            color: Colors.blue[50],
            child: Row(
              children: [
                Icon(Icons.gps_fixed, color: statusColor),
                const SizedBox(width: 10),
                Text("$statusGPS\n${currentLat?.toStringAsFixed(6)}, ${currentLng?.toStringAsFixed(6)}"),
                const Spacer(),
                ElevatedButton(onPressed: _rekamBatas, child: const Text("REKAM BATAS")),
              ],
            ),
          ),

          // List Batas
          if (_generatedGridPoints.isEmpty)
            Expanded(
              child: Column(
                children: [
                  const Padding(padding: EdgeInsets.all(8.0), child: Text("Langkah 1: Kelilingi lahan & Rekam Batas")),
                  Expanded(
                    child: ListView.builder(
                      itemCount: _perimeterPoints.length,
                      itemBuilder: (c, i) => ListTile(
                        leading: const Icon(Icons.flag, color: Colors.orange),
                        title: Text("Batas ${i+1}"),
                        subtitle: Text("${_perimeterPoints[i]['lat']?.toStringAsFixed(6)}, ..."),
                      ),
                    ),
                  ),
                ],
              ),
            ),

          // LIST GRID SAMPLING (MUNCUL SETELAH HITUNG)
          if (_generatedGridPoints.isNotEmpty)
            Expanded(
              child: Column(
                children: [
                  Container(
                      width: double.infinity,
                      color: Colors.green[100],
                      padding: const EdgeInsets.all(8.0),
                      child: const Text("Langkah 2: Datangi Titik Sampling Ini", textAlign: TextAlign.center, style: TextStyle(fontWeight: FontWeight.bold))
                  ),
                  Expanded(
                    child: ListView.builder(
                      itemCount: _generatedGridPoints.length,
                      itemBuilder: (c, i) {
                        // Hitung jarak user ke titik target
                        double dist = 0;
                        if (currentLat != null) {
                          dist = mp.SphericalUtil.computeDistanceBetween(
                              mp.LatLng(currentLat!, currentLng!),
                              _generatedGridPoints[i]
                          ).toDouble();
                        }

                        return Card(
                          margin: const EdgeInsets.symmetric(horizontal: 10, vertical: 5),
                          child: ListTile(
                            leading: CircleAvatar(child: Text("${i+1}")),
                            title: Text("Target Sampling ${i+1}"),
                            subtitle: Text("Lat: ${_generatedGridPoints[i].latitude.toStringAsFixed(6)}\nLng: ${_generatedGridPoints[i].longitude.toStringAsFixed(6)}"),
                            trailing: Column(
                              mainAxisAlignment: MainAxisAlignment.center,
                              children: [
                                Text("${dist.toStringAsFixed(1)} m", style: const TextStyle(fontWeight: FontWeight.bold)),
                                const Text("Jarak", style: TextStyle(fontSize: 10)),
                              ],
                            ),
                          ),
                        );
                      },
                    ),
                  ),
                ],
              ),
            ),

          // Footer
          Container(
            padding: const EdgeInsets.all(16),
            child: Row(
              children: [
                IconButton(onPressed: _reset, icon: const Icon(Icons.refresh, color: Colors.red)),
                const SizedBox(width: 10),
                Expanded(
                  child: ElevatedButton.icon(
                    // Tombol Nonaktif jika titik kurang dari 3
                    onPressed: _perimeterPoints.length < 3 ? null : _kalkulasiDanGenerateGrid,
                    style: ElevatedButton.styleFrom(backgroundColor: Colors.blue[800], foregroundColor: Colors.white, padding: const EdgeInsets.symmetric(vertical: 15)),
                    icon: const Icon(Icons.grid_goldenratio),
                    label: const Text("HITUNG LUAS & BUAT GRID"),
                  ),
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }
}