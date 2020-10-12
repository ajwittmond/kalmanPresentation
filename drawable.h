#pragma once

#include <gtkmm/drawingarea.h>
#include <sigc++/sigc++.h>
#include <vector>

//class to represent a hierarchy of objects drawable to a cairo canvas
//child drawables can be added and are automatically drawn
class Drawable : public sigc::trackable{
protected:
  std::vector<std::shared_ptr<Drawable>> children;

  bool drawChildren(const Cairo::RefPtr<Cairo::Context>& cr) {
    bool out = false;
    for(auto& d: children){
      out = out || d->draw(cr);
    }
    return out;
  }

public:
  Drawable() = default;

  virtual bool draw(const Cairo::RefPtr<Cairo::Context>& cr) = 0;

  //add drawable to area
  void connect(Gtk::DrawingArea& area){
    area.signal_draw().connect(sigc::mem_fun(*this, &Drawable::draw));
  }

  // references can't be null
  void addChild(std::shared_ptr<Drawable>&& d) {
    children.push_back(d);
  }

  void removeChild(int i) {
    children.erase(children.begin() + i);
  }

  const std::vector<std::shared_ptr<Drawable>>& getChildren(){
    return children;
  }
};

class ChildDrawer: public Drawable{
public:
  ChildDrawer() = default;
  bool draw(const Cairo::RefPtr<Cairo::Context>& cr) override{
    return drawChildren(cr);
  }
};

// An abstract drawable that draws some primitive cairo path with a given fill, stroke
// scale, orientation, and position
class Shape : public Drawable{
private:
  bool between(double x,double min, double max){
    return x>min && x<max;
  }

public:
  double x=0,y=0; // coordinates of center
  double angle=0;
  double fr = 0, fg = 0, fb = 0, fa = 1;
  double sr = 0, sg = 0, sb = 0, sa = 1;
  double sx=1,sy=1; //size of bounding box

  Shape() = default;

  Shape(double x, double y, double sx, double sy): x(x), y(y), sx(sx), sy(sy){}

  void set_position(double x, double y){
    this->x = x;
    this->y = y;
  }

  void set_fill_color(double r,double g, double b, double a){
    this->fr = r;
    this->fg = g;
    this->fb = b;
    this->fa = a;
  }

  void set_stroke_color(double r, double g, double b, double a) {
    this->sr = r;
    this->sg = g;
    this->sb = b;
    this->sa = a;
  }

  void set_dimensions(double sx, double sy){
    this->sx =sx; this->sy = sy;
  }

  virtual bool draw_shape(const Cairo::RefPtr<Cairo::Context> &cr) = 0;

  virtual bool draw(const Cairo::RefPtr<Cairo::Context> &cr) override{
    cr->save();
    cr->translate(x, y);
    cr->scale(sx, sy);
    cr->rotate(angle);
    cr->set_source_rgb(fr, fg, fb);
    bool out = draw_shape(cr);
    cr->restore();
    return out;
  }

  double top(){
    return y - sy/2;
  }

  double bottom(){
    return y + sy / 2;
  }

  double left(){
    return x - sx /2;
  }

  double right(){
    return y + sx/2;
  }

  bool point_in_box(double x, double y) {
    return between(x, this->x - sx, this->x + sx) &&
           between(y, this->y - sy, this->y + sy);
  }

  //by default tests the unoriented bounding box
  virtual bool point_inside(double x, double y){
    return point_in_box(x,y);
  }
};

class Square : public Shape{
public:
  Square() = default;

  Square(double x, double y, double sx, double sy) : Shape(x, y, sx, sy) {  }

  virtual bool draw_shape(const Cairo::RefPtr<Cairo::Context> &cr) override{

    cr->rectangle(-0.5, -0.5, 1, 1);
    cr->close_path();
    cr->fill();
    cr->set_source_rgb(sr, sg, sb);
    //cr->stroke();
    return false;
  }
};

class Hexagon : public Shape {
public:
  Hexagon() = default;

  Hexagon(double x, double y, double sx, double sy) : Shape(x, y, sx, sy) {}

  virtual bool draw_shape(const Cairo::RefPtr<Cairo::Context> &cr) override {
    cr->translate(-0.5, -0.5);
    cr->move_to(0.5 , 0);
    cr->line_to(1,1/(2*sqrt(3)));
    cr->line_to(1,1 - 1 / (2 * sqrt(3)));
    cr->line_to(0.5, 1);
    cr->line_to(0,1 - 1 / (2 * sqrt(3)));
    cr->line_to(0, 1 / (2 * sqrt(3)));
    cr->close_path();
    cr->fill();
    cr->set_source_rgb(sr, sg, sb);
    cr->stroke();
    return false;
  }
};

// a wrapper for a drawing area that add a convienience function
// for triggering a draw
class AreaController{
protected:
  Gtk::DrawingArea* area;
public:
  AreaController(Gtk::DrawingArea *area): area{area} {}

  void invalidate_rect() {
    auto win = area->get_window();
    if(win)
      win->invalidate_rect(area->get_allocation(), true);
  }
};
