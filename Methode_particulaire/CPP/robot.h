class Robot
{
private:
  float m_x;
  float m_y;
  float m_theta;
  float m_u;

public:
  Robot(); // Constructeur
  void Show() const; // Affichage
  //Methodes
  void evolution();
  void draw();
  float scenario();
  //setter
  void setX(float x);
  void setY(float y);
  void setTheta(float theta);
  void setU(float u);
  //getter
  int getX() const;
  int getY() const;
  int getTheta() const;
  int getU() const;

};
